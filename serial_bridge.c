/**
 * serial_bridge.c - Real-time audio streaming to Genesis Engine hardware
 *
 * This module streams YM2612 and PSG register writes over serial to real
 * Genesis audio hardware. The emulator handles all timing - we just send
 * register writes as they occur.
 *
 * Protocol (matches EmulatorBridge Arduino sketch):
 *   0x50 dd       - Write dd to PSG
 *   0x52 aa dd    - Write dd to YM2612 Port 0, register aa
 *   0x53 aa dd    - Write dd to YM2612 Port 1, register aa
 *   0x00          - PING (device responds with ACK + board type + READY)
 *   0x66          - End/reset (silence chips)
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "serial_bridge.h"

// =============================================================================
// Protocol Constants (must match EmulatorBridge Arduino sketch)
// =============================================================================

#define CMD_PING         0x00
#define CMD_ACK          0x0F
#define CMD_PSG_WRITE    0x50
#define CMD_YM2612_PORT0 0x52
#define CMD_YM2612_PORT1 0x53
#define CMD_END_STREAM   0x66
#define FLOW_READY       0x06
#define FLOW_NAK         0x15

// Chunk protocol
#define CHUNK_HEADER     0x01
#define CHUNK_SIZE       64   // Match Arduino's expectation

#define SERIAL_BAUD      1000000

// =============================================================================
// Write Buffer - chunked with flow control
// =============================================================================

#define WRITE_BUFFER_SIZE 128
static uint8_t write_buffer[WRITE_BUFFER_SIZE];
static int write_buffer_pos = 0;

// Flow control state
static bool waiting_for_ack = false;
static int ack_credits = 1;  // Start with 1 credit (can send immediately)
static bool need_retransmit = false;

// Last sent chunk (for retransmit on NAK)
static uint8_t last_chunk[CHUNK_SIZE];
static int last_chunk_len = 0;

// =============================================================================
// Platform-Specific Serial Implementation
// =============================================================================

#ifdef _WIN32

#include <windows.h>

static HANDLE serial_handle = INVALID_HANDLE_VALUE;

static bool serial_open(const char *port, int baud) {
    // Windows needs \\.\ prefix for COM ports above COM9
    char full_path[256];
    if (strncmp(port, "\\\\.\\", 4) == 0 || strncmp(port, "COM", 3) != 0) {
        strncpy(full_path, port, sizeof(full_path) - 1);
    } else {
        snprintf(full_path, sizeof(full_path), "\\\\.\\%s", port);
    }

    serial_handle = CreateFileA(
        full_path,
        GENERIC_READ | GENERIC_WRITE,
        0,              // No sharing
        NULL,           // Default security
        OPEN_EXISTING,
        0,              // No overlapped I/O
        NULL
    );

    if (serial_handle == INVALID_HANDLE_VALUE) {
        return false;
    }

    // Configure serial port
    DCB dcb = {0};
    dcb.DCBlength = sizeof(dcb);

    if (!GetCommState(serial_handle, &dcb)) {
        CloseHandle(serial_handle);
        serial_handle = INVALID_HANDLE_VALUE;
        return false;
    }

    dcb.BaudRate = baud;
    dcb.ByteSize = 8;
    dcb.StopBits = ONESTOPBIT;
    dcb.Parity = NOPARITY;
    dcb.fDtrControl = DTR_CONTROL_ENABLE;
    dcb.fRtsControl = RTS_CONTROL_ENABLE;
    dcb.fOutxCtsFlow = FALSE;
    dcb.fOutxDsrFlow = FALSE;
    dcb.fOutX = FALSE;
    dcb.fInX = FALSE;

    if (!SetCommState(serial_handle, &dcb)) {
        CloseHandle(serial_handle);
        serial_handle = INVALID_HANDLE_VALUE;
        return false;
    }

    // Set timeouts - blocking reads initially (for connection handshake)
    // Will be switched to non-blocking after connection
    COMMTIMEOUTS timeouts = {0};
    timeouts.ReadIntervalTimeout = 50;
    timeouts.ReadTotalTimeoutConstant = 1000;    // 1 sec timeout for handshake
    timeouts.ReadTotalTimeoutMultiplier = 10;
    timeouts.WriteTotalTimeoutConstant = 0;
    timeouts.WriteTotalTimeoutMultiplier = 0;
    SetCommTimeouts(serial_handle, &timeouts);

    // Increase transmit buffer size to reduce blocking
    SetupComm(serial_handle, 4096, 4096);

    // Toggle DTR to reset Arduino
    EscapeCommFunction(serial_handle, CLRDTR);
    Sleep(100);
    EscapeCommFunction(serial_handle, SETDTR);
    Sleep(2000);  // Wait for Arduino bootloader

    // Clear any garbage in buffers
    PurgeComm(serial_handle, PURGE_RXCLEAR | PURGE_TXCLEAR);

    return true;
}

static void serial_close(void) {
    if (serial_handle != INVALID_HANDLE_VALUE) {
        CloseHandle(serial_handle);
        serial_handle = INVALID_HANDLE_VALUE;
    }
}

static bool serial_is_open(void) {
    return serial_handle != INVALID_HANDLE_VALUE;
}

static void serial_set_nonblocking(void) {
    if (serial_handle == INVALID_HANDLE_VALUE) return;

    // Switch to non-blocking reads for real-time emulation
    COMMTIMEOUTS timeouts = {0};
    timeouts.ReadIntervalTimeout = MAXDWORD;     // Return immediately
    timeouts.ReadTotalTimeoutConstant = 0;
    timeouts.ReadTotalTimeoutMultiplier = 0;
    timeouts.WriteTotalTimeoutConstant = 0;
    timeouts.WriteTotalTimeoutMultiplier = 0;
    SetCommTimeouts(serial_handle, &timeouts);
}

static int serial_write_bytes(const uint8_t *data, int len) {
    if (serial_handle == INVALID_HANDLE_VALUE) return -1;

    DWORD written;
    if (!WriteFile(serial_handle, data, len, &written, NULL)) {
        return -1;
    }
    return (int)written;
}

static int serial_read_bytes(uint8_t *data, int len) {
    if (serial_handle == INVALID_HANDLE_VALUE) return -1;

    DWORD read_count;
    if (!ReadFile(serial_handle, data, len, &read_count, NULL)) {
        return -1;
    }
    return (int)read_count;
}

int serial_bridge_list_ports(char ports[][64], int max_ports) {
    int count = 0;

    // Try COM1 through COM256
    for (int i = 1; i <= 256 && count < max_ports; i++) {
        char port_name[16];
        snprintf(port_name, sizeof(port_name), "COM%d", i);

        char full_path[32];
        snprintf(full_path, sizeof(full_path), "\\\\.\\%s", port_name);

        HANDLE h = CreateFileA(
            full_path,
            GENERIC_READ | GENERIC_WRITE,
            0, NULL, OPEN_EXISTING, 0, NULL
        );

        if (h != INVALID_HANDLE_VALUE) {
            CloseHandle(h);
            strncpy(ports[count], port_name, 64);
            ports[count][63] = '\0';
            count++;
        }
    }

    return count;
}

#else  // Linux / macOS

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <dirent.h>
#include <sys/ioctl.h>

static int serial_fd = -1;

static bool serial_open(const char *port, int baud) {
    serial_fd = open(port, O_RDWR | O_NOCTTY | O_SYNC);
    if (serial_fd < 0) {
        return false;
    }

    struct termios tty;
    memset(&tty, 0, sizeof(tty));

    if (tcgetattr(serial_fd, &tty) != 0) {
        close(serial_fd);
        serial_fd = -1;
        return false;
    }

    // Set baud rate
    // Note: B1000000 may not be available on all systems
    speed_t speed;
    switch (baud) {
        case 115200:  speed = B115200; break;
        case 230400:  speed = B230400; break;
        case 460800:  speed = B460800; break;
        case 500000:  speed = B500000; break;
        case 576000:  speed = B576000; break;
        case 921600:  speed = B921600; break;
#ifdef B1000000
        case 1000000: speed = B1000000; break;
#endif
        default:      speed = B115200; break;
    }

    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);

    // 8N1, no flow control
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag &= ~(PARENB | PARODD | CSTOPB);
    tty.c_cflag |= CLOCAL | CREAD;
    tty.c_cflag &= ~CRTSCTS;

    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON | IXOFF | IXANY);
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    tty.c_oflag &= ~OPOST;

    // Read timeout
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 10;  // 1 second timeout

    if (tcsetattr(serial_fd, TCSANOW, &tty) != 0) {
        close(serial_fd);
        serial_fd = -1;
        return false;
    }

    // Toggle DTR to reset Arduino
    int status;
    if (ioctl(serial_fd, TIOCMGET, &status) == 0) {
        status &= ~TIOCM_DTR;
        ioctl(serial_fd, TIOCMSET, &status);
        usleep(100000);  // 100ms
        status |= TIOCM_DTR;
        ioctl(serial_fd, TIOCMSET, &status);
        usleep(2000000); // Wait 2s for bootloader
    }

    // Flush buffers
    tcflush(serial_fd, TCIOFLUSH);

    return true;
}

static void serial_close(void) {
    if (serial_fd >= 0) {
        close(serial_fd);
        serial_fd = -1;
    }
}

static bool serial_is_open(void) {
    return serial_fd >= 0;
}

static int serial_write_bytes(const uint8_t *data, int len) {
    if (serial_fd < 0) return -1;
    return write(serial_fd, data, len);
}

static int serial_read_bytes(uint8_t *data, int len) {
    if (serial_fd < 0) return -1;
    return read(serial_fd, data, len);
}

int serial_bridge_list_ports(char ports[][64], int max_ports) {
    int count = 0;
    DIR *dir;
    struct dirent *ent;

#ifdef __APPLE__
    // macOS: /dev/cu.usbmodem* or /dev/cu.usbserial*
    dir = opendir("/dev");
    if (dir) {
        while ((ent = readdir(dir)) != NULL && count < max_ports) {
            if (strncmp(ent->d_name, "cu.usb", 6) == 0) {
                snprintf(ports[count], 64, "/dev/%s", ent->d_name);
                count++;
            }
        }
        closedir(dir);
    }
#else
    // Linux: /dev/ttyUSB* or /dev/ttyACM*
    dir = opendir("/dev");
    if (dir) {
        while ((ent = readdir(dir)) != NULL && count < max_ports) {
            if (strncmp(ent->d_name, "ttyUSB", 6) == 0 ||
                strncmp(ent->d_name, "ttyACM", 6) == 0) {
                snprintf(ports[count], 64, "/dev/%s", ent->d_name);
                count++;
            }
        }
        closedir(dir);
    }
#endif

    return count;
}

#endif  // Platform-specific code

// =============================================================================
// Bridge State
// =============================================================================

static bool bridge_enabled = false;
static bool bridge_connected = false;
static char bridge_port[256] = "";
static uint8_t board_type = 0;

// =============================================================================
// Public API
// =============================================================================

void serial_bridge_init(void) {
    bridge_enabled = false;
    bridge_connected = false;
    bridge_port[0] = '\0';
    board_type = 0;
}

void serial_bridge_shutdown(void) {
    serial_bridge_disconnect();
}

bool serial_bridge_connect(const char *port) {
    // Disconnect if already connected
    if (bridge_connected) {
        serial_bridge_disconnect();
    }

    printf("Serial bridge: Connecting to %s...\n", port);

    if (!serial_open(port, SERIAL_BAUD)) {
        printf("Serial bridge: Failed to open %s\n", port);
        return false;
    }

    strncpy(bridge_port, port, sizeof(bridge_port) - 1);
    bridge_port[sizeof(bridge_port) - 1] = '\0';

    // Send PING and wait for ACK + board type + READY
    uint8_t ping = CMD_PING;
    serial_write_bytes(&ping, 1);

    uint8_t response[3] = {0};
    int total_read = 0;
    int attempts = 0;

    // Try to read 3 bytes with retries
    while (total_read < 3 && attempts < 10) {
        int n = serial_read_bytes(response + total_read, 3 - total_read);
        if (n > 0) {
            total_read += n;
        }
        attempts++;
    }

    if (total_read >= 3 &&
        response[0] == CMD_ACK &&
        response[2] == FLOW_READY) {

        board_type = response[1];
        bridge_connected = true;
        bridge_enabled = true;

        // Switch to non-blocking mode for real-time streaming
        serial_set_nonblocking();

        const char *board_names[] = {
            "Unknown",
            "Arduino Uno",
            "Arduino Mega",
            "Other",
            "Teensy 4.x",
            "ESP32"
        };
        const char *name = (board_type < 6) ? board_names[board_type] : "Unknown";

        printf("Serial bridge: Connected to %s on %s\n", name, port);
        return true;
    }

    printf("Serial bridge: No response from device (got %d bytes)\n", total_read);
    serial_close();
    bridge_port[0] = '\0';
    return false;
}

bool serial_bridge_auto_connect(void) {
    char ports[16][64];
    int count = serial_bridge_list_ports(ports, 16);

    if (count == 0) {
        printf("Serial bridge: No serial ports found\n");
        return false;
    }

    printf("Serial bridge: Scanning %d port(s)...\n", count);

    for (int i = 0; i < count; i++) {
        printf("  Trying %s...\n", ports[i]);

        if (serial_bridge_connect(ports[i])) {
            return true;
        }
    }

    printf("Serial bridge: No Genesis Engine board found\n");
    return false;
}

void serial_bridge_disconnect(void) {
    if (bridge_connected) {
        // Send end-of-stream to silence chips
        uint8_t cmd = CMD_END_STREAM;
        serial_write_bytes(&cmd, 1);

        serial_close();
        bridge_connected = false;
        bridge_enabled = false;
        printf("Serial bridge: Disconnected\n");
    }
}

void serial_bridge_enable(bool enable) {
    if (bridge_connected) {
        bridge_enabled = enable;
        printf("Serial bridge: %s\n", enable ? "Enabled" : "Disabled");

        if (!enable) {
            // Silence chips when disabling
            serial_bridge_reset();
        }
    }
}

bool serial_bridge_is_connected(void) {
    return bridge_connected;
}

bool serial_bridge_is_enabled(void) {
    return bridge_enabled && bridge_connected;
}

const char* serial_bridge_get_port(void) {
    return bridge_connected ? bridge_port : NULL;
}

uint8_t serial_bridge_get_board_type(void) {
    return board_type;
}

// =============================================================================
// Chunked Write Functions - With flow control like stream_vgm.py
// =============================================================================

// Check for ACK/NAK responses from Arduino (non-blocking)
static void check_flow_control(void) {
    if (!serial_is_open()) return;

    uint8_t buf[16];
    int n = serial_read_bytes(buf, sizeof(buf));
    for (int i = 0; i < n; i++) {
        if (buf[i] == FLOW_READY) {
            ack_credits++;
            waiting_for_ack = false;
            need_retransmit = false;
        } else if (buf[i] == FLOW_NAK) {
            // NAK received - need to retransmit last chunk
            ack_credits++;
            waiting_for_ack = false;
            need_retransmit = true;
        }
    }
}

// Send a chunk with header, length, data, checksum (matches stream_vgm.py)
static void send_chunk(const uint8_t *data, int len) {
    if (len <= 0 || len > CHUNK_SIZE) return;

    // Save for potential retransmit
    memcpy(last_chunk, data, len);
    last_chunk_len = len;

    // Calculate checksum: XOR of length and all data bytes
    uint8_t checksum = (uint8_t)len;
    for (int i = 0; i < len; i++) {
        checksum ^= data[i];
    }

    // Build packet: [HEADER][LENGTH][DATA...][CHECKSUM]
    uint8_t header[2] = { CHUNK_HEADER, (uint8_t)len };
    serial_write_bytes(header, 2);
    serial_write_bytes(data, len);
    serial_write_bytes(&checksum, 1);

    // Mark that we're waiting for ACK
    ack_credits--;
    waiting_for_ack = true;
}

// Retransmit last chunk (after NAK)
static void retransmit_chunk(void) {
    if (last_chunk_len <= 0) return;

    uint8_t checksum = (uint8_t)last_chunk_len;
    for (int i = 0; i < last_chunk_len; i++) {
        checksum ^= last_chunk[i];
    }

    uint8_t header[2] = { CHUNK_HEADER, (uint8_t)last_chunk_len };
    serial_write_bytes(header, 2);
    serial_write_bytes(last_chunk, last_chunk_len);
    serial_write_bytes(&checksum, 1);

    ack_credits--;
    waiting_for_ack = true;
    need_retransmit = false;
}

// Flush the write buffer as a chunk (internal)
static void flush_write_buffer(void) {
    if (write_buffer_pos <= 0 || !serial_is_open()) return;

    // Check for any pending ACKs/NAKs (non-blocking)
    check_flow_control();

    // Handle retransmit if needed (prioritize over new data)
    if (need_retransmit) {
        retransmit_chunk();
        return;  // Don't send new data until retransmit is ACK'd
    }

    // Send the chunk - never block, Arduino has a ring buffer
    send_chunk(write_buffer, write_buffer_pos);
    write_buffer_pos = 0;
}

// Public flush function - call once per frame from genesis.c
void serial_bridge_flush(void) {
    if (bridge_enabled) {
        flush_write_buffer();
        // Also check for any pending ACKs
        check_flow_control();
    }
}

// Add bytes to write buffer, flush when full
static void buffered_write(const uint8_t *data, int len) {
    // If this write would exceed chunk size, flush first
    if (write_buffer_pos + len > CHUNK_SIZE) {
        flush_write_buffer();
    }

    // Add to buffer
    memcpy(write_buffer + write_buffer_pos, data, len);
    write_buffer_pos += len;

    // Auto-flush when buffer reaches chunk size
    if (write_buffer_pos >= CHUNK_SIZE) {
        flush_write_buffer();
    }
}

// =============================================================================
// Audio Write Functions - Called from YM2612/PSG emulation
// =============================================================================

void serial_bridge_ym2612_write(uint8_t port, uint8_t reg, uint8_t value) {
    if (!bridge_enabled || !serial_is_open()) return;

    uint8_t cmd[3] = {
        (port == 0) ? CMD_YM2612_PORT0 : CMD_YM2612_PORT1,
        reg,
        value
    };
    buffered_write(cmd, 3);
}

void serial_bridge_psg_write(uint8_t value) {
    if (!bridge_enabled || !serial_is_open()) return;

    uint8_t cmd[2] = { CMD_PSG_WRITE, value };
    buffered_write(cmd, 2);
}

void serial_bridge_reset(void) {
    if (!serial_is_open()) return;

    // Flush any pending writes first
    flush_write_buffer();

    uint8_t cmd = CMD_END_STREAM;
    serial_write_bytes(&cmd, 1);

    // Drain any pending response
    uint8_t buf[16];
    serial_read_bytes(buf, sizeof(buf));
}
