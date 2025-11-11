const std = @import("std");
const CPU = @import("cpu.zig").CPU;

pub fn main() !void {
    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    defer {
        const leaked = gpa.deinit();
        if (leaked == .leak) {
            std.log.err("Memory leak detected", .{});
        }
    }

    const allocator = gpa.allocator();
    const cpu = try allocator.create(CPU);
    defer allocator.destroy(cpu);

    cpu.* = .{};

    std.debug.print("CPU allocated at address: {*}\n", .{cpu});
}

fn emuLoop(cpu: *CPU) !void {
    const running = true;

    while (running) {
        cpu.step();
    }
}