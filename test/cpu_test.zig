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

test "DAA Addition 0x15 + 0x27 = 0x42" {
    var cpu: CPU = .{};
    // LD A, 0x15; ADD A, 0x27; DAA
    const init_mem = [_]u8{ 0x3E, 0x15, 0xC6, 0x27, 0x27 };
    run_test(&cpu, &init_mem, 3);
    try testing.expectEqual(@as(u8, 0x42), cpu.a);
    try testing.expect(!cpu.readFlag(.h));
}

test "DAA Addition Carry 0x83 + 0x91 = 0x74 (Carry)" {
    var cpu: CPU = .{};
    // LD A, 0x83; ADD A, 0x91; DAA
    const init_mem = [_]u8{ 0x3E, 0x83, 0xC6, 0x91, 0x27 };
    run_test(&cpu, &init_mem, 3);
    try testing.expectEqual(@as(u8, 0x74), cpu.a);
    try testing.expect(cpu.readFlag(.c));
}

test "DAA Subtraction 0x42 - 0x15 = 0x27" {
    var cpu: CPU = .{};
    // LD A, 0x42; LD B, 0x15; SUB B; DAA
    const init_mem = [_]u8{ 0x3E, 0x42, 0x06, 0x15, 0x90, 0x27 };
    run_test(&cpu, &init_mem, 4);
    try testing.expectEqual(@as(u8, 0x27), cpu.a);
}

test "DAA Subtraction Carry 0x15 - 0x27 = 0x88 (Carry)" {
    var cpu: CPU = .{};
    // LD A, 0x15; LD B, 0x27; SUB B; DAA
    const init_mem = [_]u8{ 0x3E, 0x15, 0x06, 0x27, 0x90, 0x27 };
    run_test(&cpu, &init_mem, 4);
    try testing.expectEqual(@as(u8, 0x88), cpu.a);
    try testing.expect(cpu.readFlag(.c));
}

test "DAA Zero Result" {
    var cpu: CPU = .{};
    // XOR A; DAA
    const init_mem = [_]u8{ 0xAF, 0x27 };
    run_test(&cpu, &init_mem, 2);
    try testing.expectEqual(@as(u8, 0x00), cpu.a);
    try testing.expect(cpu.readFlag(.z));
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
