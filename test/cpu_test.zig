const std = @import("std");
const testing = std.testing;
const Cpu = @import("cpu").Cpu;
const Mmu = @import("mmu").Mmu;
const Mbc = @import("mbc").Mbc;
const Timer = @import("timer").Timer;
const Joypad = @import("joypad").Joypad;
const Io = @import("io").Io;
const Interrupts = @import("interrupts").Interrupts;
const hw = @import("hw");

const TestContext = struct {
    arena: std.heap.ArenaAllocator,
    cpu: *Cpu,

    pub fn init() !TestContext {
        var arena = std.heap.ArenaAllocator.init(testing.allocator);
        errdefer arena.deinit();
        const aa = arena.allocator();

        const interrupts = try aa.create(Interrupts);
        interrupts.* = .{};

        const timer = try aa.create(Timer);
        timer.* = .{ .interrupts = interrupts };

        const joypad = try aa.create(Joypad);
        joypad.* = .{};

        const io = try aa.create(Io);
        io.* = .{
            .timer = timer,
            .joypad = joypad,
            .interrupts = interrupts,
        };

        const rom_data = try aa.create([0x8000]u8);
        @memset(rom_data, 0);

        const mbc = try aa.create(Mbc);
        mbc.* = .{ .mbc0 = .{ .rom = rom_data } };

        const mmu = try aa.create(Mmu);
        mmu.* = .{
            .interrupts = interrupts,
            .io = io,
            .mbc = mbc,
        };

        const cpu = try aa.create(Cpu);
        cpu.* = .{
            .mmu = mmu,
            .interrupts = interrupts,
            .timer = timer,
        };

        return .{
            .arena = arena,
            .cpu = cpu,
        };
    }

    pub fn deinit(self: *TestContext) void {
        self.arena.deinit();
    }
};

test "LD b n8" {
    var ctx = try TestContext.init();
    defer ctx.deinit();
    const init_mem = [_]u8{ 0x06, 0x69 };
    run_test(ctx.cpu, &init_mem, 1);
    try testing.expectEqual(@as(u8, 0x69), ctx.cpu.b);
}

test "SWAP B" {
    var ctx = try TestContext.init();
    defer ctx.deinit();
    const init_mem = [_]u8{ 0x06, 0b10000011, 0xCB, 0x30 };
    run_test(ctx.cpu, &init_mem, 3);
    try testing.expectEqual(@as(u8, 0b00111000), ctx.cpu.b);
}

test "DAA Addition 0x15 + 0x27 = 0x42" {
    var ctx = try TestContext.init();
    defer ctx.deinit();
    // LD A, 0x15; ADD A, 0x27; DAA
    const init_mem = [_]u8{ 0x3E, 0x15, 0xC6, 0x27, 0x27 };
    run_test(ctx.cpu, &init_mem, 3);
    try testing.expectEqual(@as(u8, 0x42), ctx.cpu.a);
    try testing.expect(!ctx.cpu.readFlag(.h));
}

test "DAA Addition Carry 0x83 + 0x91 = 0x74 (Carry)" {
    var ctx = try TestContext.init();
    defer ctx.deinit();
    // LD A, 0x83; ADD A, 0x91; DAA
    const init_mem = [_]u8{ 0x3E, 0x83, 0xC6, 0x91, 0x27 };
    run_test(ctx.cpu, &init_mem, 3);
    try testing.expectEqual(@as(u8, 0x74), ctx.cpu.a);
    try testing.expect(ctx.cpu.readFlag(.c));
}

test "DAA Subtraction 0x42 - 0x15 = 0x27" {
    var ctx = try TestContext.init();
    defer ctx.deinit();
    // LD A, 0x42; LD B, 0x15; SUB B; DAA
    const init_mem = [_]u8{ 0x3E, 0x42, 0x06, 0x15, 0x90, 0x27 };
    run_test(ctx.cpu, &init_mem, 4);
    try testing.expectEqual(@as(u8, 0x27), ctx.cpu.a);
}

test "DAA Subtraction Carry 0x15 - 0x27 = 0x88 (Carry)" {
    var ctx = try TestContext.init();
    defer ctx.deinit();
    // LD A, 0x15; LD B, 0x27; SUB B; DAA
    const init_mem = [_]u8{ 0x3E, 0x15, 0x06, 0x27, 0x90, 0x27 };
    run_test(ctx.cpu, &init_mem, 4);
    try testing.expectEqual(@as(u8, 0x88), ctx.cpu.a);
    try testing.expect(ctx.cpu.readFlag(.c));
}

test "DAA Zero Result" {
    var ctx = try TestContext.init();
    defer ctx.deinit();
    // XOR A; DAA
    const init_mem = [_]u8{ 0xAF, 0x27 };
    run_test(ctx.cpu, &init_mem, 2);
    try testing.expectEqual(@as(u8, 0x00), ctx.cpu.a);
    try testing.expect(ctx.cpu.readFlag(.z));
}

fn run_test(cpu: *Cpu, mem: []const u8, steps: u16) void {
    const start_address = hw.Map.wram0.start;
    var i: u16 = 0;

    cpu.boot();
    cpu.pc = hw.Map.wram0.start;

    while (i < mem.len) : (i += 1) {
        cpu.mmu.write(start_address + i, mem[i]);
    }

    i = 0;
    while (i < steps) : (i += 1) {
        cpu.step();
    }
}
