# ZigGBemu

A Game Boy emulator implemented in Zig.

## Project Overview

ZigGBemu is a modular Game Boy emulator focusing on performance and code clarity using Zig's unique features. It aims to provide a complete emulation of the original Game Boy hardware.

### Key Technologies
- **Zig:** The primary programming language.
- **Zig Build System:** Manages building, running, and testing the project.
- **Comptime:** Extensively used for instruction decoding and generation in the CPU.

### Architecture

The project follows a modular design, separating the core components of the Game Boy hardware:

- **`src/main.zig`**: Entry point. Initializes the allocator and the emulator instance.
- **`src/emulator.zig`**: The high-level orchestrator that manages the lifecycle of the CPU and Memory Management Unit (MMU). It now initializes and integrates the PPU and SDT.
- **`src/cpu.zig`**: Implementation of the LR35902 CPU. It uses `comptime` function generators to create instruction handlers, minimizing runtime overhead.
- **`src/ppu.zig`**: Implementation of the Pixel Processing Unit. Features a mode-based state machine (Modes 0-3), a pixel fetcher, and support for BG, Window, and Sprite rendering.
- **`src/pixel_fifo.zig`**: Implements the Background and Sprite FIFOs used by the PPU's pixel fetcher to manage scanline rendering.
- **`src/display.zig`**: Manages the emulator's frame buffer and pixel output.
- **`src/mmu.zig`**: Handles the Game Boy's 64KB memory map, including VRAM and OAM mapping with appropriate hardware-level locking.
- **`src/timer.zig`**: Manages the Game Boy's internal timers and orchestrates the ticking of the PPU and SDT to maintain cycle accuracy.
- **`src/sdt.zig`**: A stub implementation of the Serial Data Transfer (SDT) functionality.
- **`src/mbc.zig` & `src/mbc*.zig`**: Implements Memory Bank Controllers (MBC0, MBC1) for handling ROM and RAM banking.
- **`src/constants.zig`**: Contains hardware-specific constants, including memory addresses, register offsets, and interrupt vectors.
- **`test/`**: Contains unit tests for verifying core components like CPU instructions and memory controllers.

## Building and Running

The project uses the standard Zig build system.

### Build the project
```bash
zig build
```

### Run the emulator
```bash
zig build run
```

### Run tests
```bash
zig build test
```

## Development Conventions

- **Modular Design**: Keep hardware components (CPU, MMU, PPU, etc.) in separate files and modules.
- **Comptime Optimization**: Leverage Zig's `comptime` capabilities for instruction decoding and table generation.
- **Error Handling**: Use Zig's standard error handling (`!T`, `try`, `catch`) consistently.
- **Memory Management**: 
    - Use `GeneralPurposeAllocator` for the main application lifecycle.
    - Use `ArenaAllocator` (via `Emulator.init`) for emulator-specific allocations to ensure easy cleanup.
- **Naming Conventions**: Follow standard Zig naming conventions (PascalCase for types, camelCase for functions and variables).
- **Testing**: Add unit tests for new instructions or hardware components in the `test/` directory.

## Current Status

- **CPU**: Core instruction set fully implemented via `comptime` generators.
- **MMU**: Memory mapping, interrupt system, and hardware register handling are functional.
- **PPU**: Mode-based rendering with pixel FIFOs implemented. Supports BG, Window, and Sprite rendering. Hardware locking for VRAM/OAM is active.
- **SDT**: Basic stub for Serial Data Transfer implemented (prints to stdout).
- **MBC**: MBC0 and MBC1 support implemented.
- **Input**: Joypad state handling is functional in the MMU.
- **Timer**: Fully integrated with the main loop, ticking the PPU and SDT to maintain synchronization.
- **Audio**: APU component is still missing.
- **Video Output**: `display.zig` exists but `drawPixel` logic needs to be connected to a GUI backend (e.g., SDL or Mach).
