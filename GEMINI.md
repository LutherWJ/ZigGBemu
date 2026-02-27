# ZigGBemu

A Game Boy emulator implemented in Zig.

## Project Overview

ZigGBemu is a project focused on building a complete Game Boy emulator using the Zig programming language. The core architecture is designed to be modular, with separate components for the CPU, Memory Management Unit (MMU), and Memory Bank Controllers (MBC).

### Key Technologies
- **Zig:** The primary programming language used for the entire project.
- **Zig Build System:** Used for building, running, and testing the emulator.

### Architecture
- **`src/main.zig`**: The entry point of the emulator. It handles memory allocation for the CPU and initializes the emulation loop.
- **`src/cpu.zig`**: Implements the LR35902 CPU instruction set. It uses Zig's `comptime` features to efficiently generate instruction handlers.
- **`src/mmu.zig`**: Manages the Game Boy's memory map, handles memory reads/writes, and manages interrupt requests and acknowledgments.
- **`src/mbc1.zig`**: A placeholder for the MBC1 (Memory Bank Controller 1) implementation, which will handle ROM and RAM banking for larger games.
- **`test/`**: Contains unit tests for the emulator components (e.g., `cpu_test.zig`).

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

- **Modular Design**: Keep the CPU, MMU, and other components in separate files and modules to maintain clarity and testability.
- **Comptime Optimization**: Leverage Zig's `comptime` capabilities for instruction decoding and generation to minimize runtime overhead.
- **Error Handling**: Use Zig's error handling patterns (`!void`, `try`, `catch`) consistently across the codebase.
- **Memory Management**: Use the `GeneralPurposeAllocator` for heap allocations and ensure all resources are properly deallocated.
- **Testing**: Add unit tests for new instructions and components in the `test/` directory.

## Current Status

- CPU instruction set is partially implemented.
- MMU handles basic memory regions and interrupts.
- MBC1 implementation is a placeholder.
- Video and Audio components are yet to be implemented.
