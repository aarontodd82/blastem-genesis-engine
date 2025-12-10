# BlastEm Hardware Audio Bridge

This is a modified version of [BlastEm](https://www.retrodev.com/blastem/) that streams real-time audio register writes to Genesis Engine hardware for authentic audio playback through real YM2612 and SN76489 chips.

## How It Works

BlastEm emulates the Genesis/Mega Drive with cycle-accurate timing. When the emulated CPU writes to the YM2612 (FM synthesizer) or SN76489 (PSG), we intercept those writes and stream them over serial using VGM-style timing commands to a microcontroller connected to real audio chips.

```
BlastEm (emulation)          Serial (1Mbaud)          Genesis Engine Board
┌─────────────────┐                                  ┌──────────────────┐
│   YM2612 write  │ ──── [wait] 0x52 reg val ───────>│ Real YM2612 chip │
│   PSG write     │ ──── [wait] 0x50 val ───────────>│ Real SN76489 PSG │
└─────────────────┘                                  └──────────────────┘
         │
         └── VGM-style wait commands encode timing at 44.1kHz sample resolution
```

## Building

### Prerequisites

- GCC or compatible C compiler
- SDL2 development libraries
- GLEW development libraries (unless building with `NO_GL=1`)

#### Windows (MSYS2/MinGW)
```bash
pacman -S mingw-w64-x86_64-gcc mingw-w64-x86_64-SDL2 mingw-w64-x86_64-glew
```

#### Linux (Debian/Ubuntu)
```bash
sudo apt install build-essential libsdl2-dev libglew-dev
```

#### macOS (Homebrew)
```bash
brew install sdl2 glew
```

### Build Commands

```bash
cd blastem

# Standard build
make

# Windows cross-compile (from Linux)
make OS=Windows

# Without OpenGL (simpler build)
make NO_GL=1
```

## Hardware Setup

1. Upload the `EmulatorBridge` sketch to your Genesis Engine board:
   - Located at: `GenesisEngine/examples/EmulatorBridge/EmulatorBridge.ino`
   - Requires the GenesisBoard library

2. Connect the board via USB

## Usage

### Keyboard Shortcuts

| Key | Action |
|-----|--------|
| `J` | Connect/disconnect hardware audio (auto-detect) |
| `H` | Toggle hardware audio on/off (when connected) |
| `M` | Toggle VGM logging (existing BlastEm feature) |

### Command Line

```bash
# Run normally, then press J to connect
./blastem game.bin

# The emulator will scan for Genesis Engine boards and connect automatically
```

### Configuration

You can also set up bindings in your `blastem.cfg`:

```
bindings {
    keys {
        h ui.toggle_hw_audio
        j ui.hw_audio_connect
    }
}
```

## Protocol

The bridge uses VGM-compatible command bytes with timing:

### Audio Commands
| Command | Format | Description |
|---------|--------|-------------|
| `0x50` | `0x50 <value>` | Write to PSG |
| `0x52` | `0x52 <reg> <value>` | Write to YM2612 Port 0 |
| `0x53` | `0x53 <reg> <value>` | Write to YM2612 Port 1 |

### Timing Commands
| Command | Format | Description |
|---------|--------|-------------|
| `0x61` | `0x61 <lo> <hi>` | Wait N samples (16-bit little-endian, 44.1kHz) |
| `0x62` | `0x62` | Wait 735 samples (1/60 sec) |
| `0x63` | `0x63` | Wait 882 samples (1/50 sec) |
| `0x70-0x7F` | single byte | Wait 1-16 samples (0x70=1, 0x7F=16) |

### Control Commands
| Command | Format | Description |
|---------|--------|-------------|
| `0xAA` | `0xAA` | PING (connection test) |
| `0x66` | `0x66` | End stream / reset hardware |

The board responds to PING with: `0x0F <board_type> 0x06` (ACK + type + READY)

## Files Modified

From the original BlastEm source:

- `serial_bridge.c` / `serial_bridge.h` - **NEW**: Serial communication and VGM-style timing
- `ym2612.c` - Added `serial_bridge_ym2612_write()` calls
- `psg.c` - Added `serial_bridge_psg_write()` call
- `genesis.c` - Added cycle adjustment sync, silence on pause/menu
- `render_sdl.c` - Added disconnect on application close
- `bindings.c` - Added UI actions for hardware audio toggle/connect
- `default.cfg` - Added default key bindings (H and J)
- `Makefile` - Added `serial_bridge.o` to AUDIOOBJS

## Troubleshooting

### No connection / "No Genesis Engine board found"
- Check USB cable and board power
- Verify the EmulatorBridge sketch is uploaded
- On Linux, add user to `dialout` group: `sudo usermod -a -G dialout $USER`
- Check available ports: `ls /dev/ttyUSB* /dev/ttyACM*` (Linux) or Device Manager (Windows)

### Audio glitches
- Ensure BlastEm is running at normal speed (not turbo/fast-forward)
- Try a different USB port (avoid USB hubs)
- Check for other programs using the serial port

### Hanging notes when pausing
- Notes are automatically silenced when entering menus or pausing
- Closing the application sends a reset command to silence hardware
- Press J to manually disconnect if needed

## License

BlastEm is licensed under the GNU GPL v3. This modification is provided under the same license.

## Credits

- [BlastEm](https://www.retrodev.com/blastem/) by Mike Pavone
- Genesis Engine hardware and library by the FM-90s project
