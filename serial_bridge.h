/**
 * serial_bridge.h - Real-time audio streaming to Genesis Engine hardware
 *
 * This module streams YM2612 and PSG register writes over serial to real
 * Genesis audio hardware. The emulator handles all timing - we just send
 * register writes as they occur.
 */

#ifndef SERIAL_BRIDGE_H
#define SERIAL_BRIDGE_H

#include <stdint.h>
#include <stdbool.h>

// Initialize the serial bridge subsystem
// master_clock is the Genesis master clock frequency (e.g., 53693175 for NTSC)
void serial_bridge_init(uint32_t master_clock);

// Shutdown and cleanup
void serial_bridge_shutdown(void);

// Connect to a specific port (e.g., "COM3" or "/dev/ttyUSB0")
bool serial_bridge_connect(const char *port);

// Auto-detect and connect to Genesis Engine board
bool serial_bridge_auto_connect(void);

// Disconnect from hardware
void serial_bridge_disconnect(void);

// Enable/disable streaming (while staying connected)
void serial_bridge_enable(bool enable);

// Status queries
bool serial_bridge_is_connected(void);
bool serial_bridge_is_enabled(void);
const char* serial_bridge_get_port(void);
uint8_t serial_bridge_get_board_type(void);

// Get list of available serial ports
// Returns number of ports found, fills ports array with port names
int serial_bridge_list_ports(char ports[][64], int max_ports);

// Audio write functions - called from chip emulation
// cycle is the current emulator cycle count (master clock cycles)
void serial_bridge_ym2612_write(uint8_t port, uint8_t reg, uint8_t value, uint32_t cycle);
void serial_bridge_psg_write(uint8_t value, uint32_t cycle);

// Flush buffered writes - call once per frame
void serial_bridge_flush(void);

// Reset/silence hardware (called on ROM change, emulator reset, etc.)
void serial_bridge_reset(void);

// Silence all channels without resetting (preserves patch data)
void serial_bridge_silence(void);

// Adjust cycle counter when emulator adjusts cycles (prevents timing drift)
void serial_bridge_adjust_cycles(uint32_t deduction);

#endif // SERIAL_BRIDGE_H
