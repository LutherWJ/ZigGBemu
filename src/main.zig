const std = @import("std");
const builtin = @import("builtin");
const Emulator = @import("emulator").Emulator;

pub fn main() !void {
    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    defer {
        const deinit_status = gpa.deinit();
        if (deinit_status == .leak) @panic("Memory Leak detected");
    }
    const allocator = gpa.allocator();

    try testRom(allocator);
}

fn testRom(allocator: std.mem.Allocator) !void {
    const rom_path = "/home/luther/Downloads/Legend of Zelda, The - Link's Awakening (USA, Europe) (Rev 2)/Legend of Zelda, The - Link's Awakening (USA, Europe) (Rev 2).gb";

    const file = try std.fs.cwd().openFile(rom_path, .{});
    defer file.close();

    const rom_size = try file.getEndPos();
    const rom_buf = try allocator.alloc(u8, rom_size);
    defer allocator.free(rom_buf);

    _ = try file.readAll(rom_buf);

    const emu = try Emulator.init(allocator, rom_buf);
    defer emu.deinit();

    std.debug.print("Starting emulation of {s}...\n", .{rom_path});

    // Simple infinite loop for now
    while (true) {
        emu.runFrame();
        // VSync/Timer here soon
    }
}
