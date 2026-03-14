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
- **`src/emulator.zig`**: The high-level orchestrator that manages the lifecycle of the CPU and Memory Management Unit (MMU).
- **`src/cpu.zig`**: Implementation of the LR35902 CPU. It uses `comptime` function generators to create instruction handlers, minimizing runtime overhead.
- **`src/mmu.zig`**: Handles the Game Boy's 64KB memory map, interrupt requests/acknowledgments, and joypad state.
- **`src/mbc.zig` & `src/mbc*.zig`**: Implements Memory Bank Controllers (MBC0, MBC1) for handling ROM and RAM banking.
- **`src/constants.zig`**: Contains hardware-specific constants, including memory addresses, register offsets, and interrupt vectors.
- **`test/`**: Contains unit tests, specifically `cpu_test.zig` for verifying instruction correctness.

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

- **CPU**: Core instruction set implemented via `comptime` generators. Supports CB-prefix instructions.
- **MMU**: Basic memory mapping and interrupt system implemented.
- **MBC**: MBC0 and MBC1 support implemented.
- **Input**: Joypad state handling is present in the MMU.
- **Video/Audio**: PPU and APU components are currently missing or under development.
- **Timer**: `timer.zig` exists but integration with the main loop needs verification.
