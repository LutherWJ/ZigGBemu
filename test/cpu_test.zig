const std = @import("std");
const cpu = @import("cpu");

test "NOP instruction" {
    var c = cpu.CPU{};
    const initial_pc = c.pc;
    c.memory[0] = 0x00;

    try c.step();

    try std.testing.expectEqual(initial_pc + 1, c.pc);
}

test "LD (BC), A instruction" {
    var c = cpu.CPU{};
    c.a = 0x42;
    c.b = 0x80;
    c.c = 0x00;
    c.memory[0] = 0x02;

    try c.step();

    try std.testing.expectEqual(@as(u8, 0x42), c.memory[0x8000]);
    try std.testing.expectEqual(@as(u16, 1), c.pc);
}

test "LD B, n instruction" {
    var c = cpu.CPU{};
    c.memory[0] = 0x06;
    c.memory[1] = 0x42;

    try c.step();

    try std.testing.expectEqual(@as(u8, 0x42), c.b);
    try std.testing.expectEqual(@as(u16, 2), c.pc);
}

test "LD (DE), A instruction" {
    var c = cpu.CPU{};
    c.a = 0x55;
    c.d = 0x90;
    c.e = 0x10;
    c.memory[0] = 0x12;

    try c.step();

    try std.testing.expectEqual(@as(u8, 0x55), c.memory[0x9010]);
    try std.testing.expectEqual(@as(u16, 1), c.pc);
}

test "LD BC, d16 instruction" {
    var c = cpu.CPU{};
    c.memory[0] = 0x01;
    c.memory[1] = 0x34; // Low byte
    c.memory[2] = 0x12; // High byte

    try c.step();

    try std.testing.expectEqual(@as(u8, 0x12), c.b);
    try std.testing.expectEqual(@as(u8, 0x34), c.c);
    try std.testing.expectEqual(@as(u16, 3), c.pc);
}

test "LD A, (BC) instruction" {
    var c = cpu.CPU{};
    c.b = 0x80;
    c.c = 0x50;
    c.memory[0] = 0x0A;
    c.memory[0x8050] = 0x99;

    try c.step();

    try std.testing.expectEqual(@as(u8, 0x99), c.a);
    try std.testing.expectEqual(@as(u16, 1), c.pc);
}

test "LD C, d8 instruction" {
    var c = cpu.CPU{};
    c.memory[0] = 0x0E;
    c.memory[1] = 0x77;

    try c.step();

    try std.testing.expectEqual(@as(u8, 0x77), c.c);
    try std.testing.expectEqual(@as(u16, 2), c.pc);
}

test "LD D, d8 instruction (0x16)" {
    var c = cpu.CPU{};
    c.memory[0] = 0x16;
    c.memory[1] = 0xAA;

    try c.step();

    try std.testing.expectEqual(@as(u8, 0xAA), c.d);
    try std.testing.expectEqual(@as(u16, 2), c.pc);
}

test "NOP instruction advances PC correctly" {
    var c = cpu.CPU{};
    c.pc = 0x1000;
    c.memory[0x1000] = 0x00;

    try c.step();

    try std.testing.expectEqual(@as(u16, 0x1001), c.pc);
}

test "LD (BC), A with different addresses" {
    var c = cpu.CPU{};
    c.a = 0xFF;
    c.b = 0x20;
    c.c = 0x30;
    c.memory[0] = 0x02;

    try c.step();

    try std.testing.expectEqual(@as(u8, 0xFF), c.memory[0x2030]);
    try std.testing.expectEqual(@as(u16, 1), c.pc);
}

test "LD B, n with boundary values" {
    var c = cpu.CPU{};
    c.memory[0] = 0x06;
    c.memory[1] = 0x00; // Test zero value

    try c.step();

    try std.testing.expectEqual(@as(u8, 0x00), c.b);
    try std.testing.expectEqual(@as(u16, 2), c.pc);

    // Test with 0xFF
    c.pc = 0;
    c.memory[1] = 0xFF;

    try c.step();

    try std.testing.expectEqual(@as(u8, 0xFF), c.b);
    try std.testing.expectEqual(@as(u16, 2), c.pc);
}

test "Multiple instructions in sequence" {
    var c = cpu.CPU{};

    // Set up a sequence: LD B, 0x10 -> LD C, 0x20 -> LD (BC), A
    c.a = 0x88;
    c.memory[0] = 0x06; // LD B, n
    c.memory[1] = 0x10;
    c.memory[2] = 0x0E; // LD C, n
    c.memory[3] = 0x20;
    c.memory[4] = 0x02; // LD (BC), A

    try c.step(); // LD B, 0x10
    try std.testing.expectEqual(@as(u8, 0x10), c.b);
    try std.testing.expectEqual(@as(u16, 2), c.pc);

    try c.step(); // LD C, 0x20
    try std.testing.expectEqual(@as(u8, 0x20), c.c);
    try std.testing.expectEqual(@as(u16, 4), c.pc);

    try c.step(); // LD (BC), A
    try std.testing.expectEqual(@as(u8, 0x88), c.memory[0x1020]);
    try std.testing.expectEqual(@as(u16, 5), c.pc);
}