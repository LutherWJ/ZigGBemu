const std = @import("std");
const builtin = @import("builtin");
const Emulator = @import("emulator").Emulator;

pub fn panic(msg: []const u8, _: ?*std.builtin.StackTrace, _: ?usize) noreturn {
    std.debug.print("\n FATAL ERROR:\n{s}\n", .{msg});
    std.process.exit(1);
}

pub fn main() !void {
    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    defer {
        const deinit_status = gpa.deinit();
        if (deinit_status == .leak) @panic("Memory Leak detected");
    }
    const allocator = gpa.allocator();

    try test_rom(allocator);
}

fn test_rom(allocator: std.mem.Allocator) !void {
    const rom_path = "/home/luther/Downloads/Legend of Zelda, The - Link's Awakening (USA, Europe) (Rev 2)/Legend of Zelda, The - Link's Awakening (USA, Europe) (Rev 2).gb";

    const file = try std.fs.cwd().openFile(rom_path, .{});
    defer file.close();

    const rom_size = try file.getEndPos();
    const rom_buf = try allocator.alloc(u8, rom_size);
    defer allocator.free(rom_buf);

    _ = try file.readAll(rom_buf);

    const emu = try Emulator.init(allocator, rom_buf);
    defer emu.deinit();

    std.debug.print("Starting emulation of {s}...\n\n\n", .{rom_path});

    var num_frames: usize = 0;
    var accumulator: i128 = 0;
    const NUM_FRAMES = 10000;

    while (num_frames < NUM_FRAMES) : (num_frames += 1) {
        const start = std.time.nanoTimestamp();
        emu.runFrame();
        const end = std.time.nanoTimestamp();
        accumulator +|= end - start;
    }

    const average_ns = @as(f128, @floatFromInt(accumulator)) / @as(f128, @floatFromInt(NUM_FRAMES));
    const average_ms = average_ns / 1_000_000.0;

    std.debug.print("Total time to execute {d} frames: {} \n", .{ NUM_FRAMES, std.fmt.fmtDuration(@intCast(accumulator)) });
    std.debug.print("Average frametime: {d:.3} ms\n", .{average_ms});
}
