// Exposes the emulator interface to the C ABI
// Can be used for WASM as well.
const std = @import("std");
const Emulator = @import("emulator.zig").Emulator;

var emu: Emulator = undefined;
const allocator = std.heap.page_allocator;

export fn init_emulator(rom_ptr: [*]const u8, rom_size: usize) i32 {
    const rom = rom_ptr[0..rom_size];
    emu = try Emulator.init(allocator, rom) catch return 1;
    return 0;
}

export fn run_frame() void {
    emu.runFrame();
}

export fn deinit_emulator() void {
    emu.deinit();
}
