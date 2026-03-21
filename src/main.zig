const std = @import("std");
const Emulator = @import("emulator.zig").Emulator;

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
    var args_iterator = try std.process.argsWithAllocator(allocator);
    defer args_iterator.deinit();

    _ = args_iterator.next(); // skip exe name
    const rom_path = args_iterator.next() orelse {
        std.debug.print("Usage: ZigGBemu <path_to_rom>\n", .{});
        return;
    };

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
        // In a real app we'd sync with VSync/Timer here
    }
}
