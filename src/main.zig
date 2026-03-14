const std = @import("std");
const Emulator = @import("emulator.zig").Emulator;

pub fn main() !void {
    var gpa = std.heap.GeneralPurposeAllocator(.{}){};

    defer {
        const deinit_status = gpa.deinit();
        if (deinit_status == .leak) @panic("Memory Leak detected");
    }

    const allocator = gpa.allocator();

    const rom = try allocator.alloc(u8, 0x4000);
    const emu = try Emulator.init(allocator, rom);
    allocator.free(rom);

    _ = emu;
}
