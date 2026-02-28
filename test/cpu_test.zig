const std = @import("std");
const testing = std.testing;
const CPU = @import("cpu").CPU;

test "LD b n8" {
    var cpu: CPU = .{};
    const init_mem = [_]u8{ 0x06, 0x69 };
    run_test(&cpu, &init_mem, 1);
    try testing.expectEqual(@as(u8, 0x69), cpu.b);
}

test "SWAP B" {
    var cpu: CPU = .{};
    const init_mem = [_]u8{ 0x06, 0b10000011, 0xCB, 0x30 };
    run_test(&cpu, &init_mem, 3);
    try testing.expectEqual(@as(u8, 0b00111000), cpu.b);
}

fn run_test(cpu: *CPU, mem: []const u8, steps: u16) void {
    const start_address = 0xC000;
    var i: u16 = 0;

    cpu.boot();
    cpu.pc = 0xC000;

    while (i < mem.len) : (i += 1) {
        cpu.memory.write(start_address + i, mem[i]);
    }

    i = 0;
    while (i < steps) : (i += 1) {
        cpu.step() catch unreachable;
    }
}
