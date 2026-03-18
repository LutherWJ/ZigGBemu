const std = @import("std");
const testing = std.testing;
const Timer = @import("timer").Timer;
const Interrupts = @import("interrupts").Interrupts;
const hw = @import("hw");

fn setup() !struct { arena: std.heap.ArenaAllocator, timer: *Timer, interrupts: *Interrupts } {
    var arena = std.heap.ArenaAllocator.init(testing.allocator);
    errdefer arena.deinit();
    const aa = arena.allocator();

    const interrupts = try aa.create(Interrupts);
    interrupts.* = .{
        .ie = 0xFF,
        .ifr = 0,
    };

    const timer = try aa.create(Timer);
    timer.* = .{
        .interrupts = interrupts,
    };

    return .{ .arena = arena, .timer = timer, .interrupts = interrupts };
}

test "DIV resets internal counter and register" {
    var ctx = try setup();
    defer ctx.arena.deinit();

    // Tick it for a bit
    ctx.timer.tick(256 * 4); // 256 M-cycles = 1024 T-cycles
    try testing.expect(ctx.timer.readDiv() > 0);

    // Write to DIV should reset it
    ctx.timer.writeDiv();
    try testing.expectEqual(@as(u8, 0), ctx.timer.readDiv());
    try testing.expectEqual(@as(u16, 0), ctx.timer.counter);
}

test "TAC unused bits return 1s" {
    var ctx = try setup();
    defer ctx.arena.deinit();

    ctx.timer.writeTac(0x00); // Set speed 4096Hz, disabled
    // Bits 3-7 should be 1
    try testing.expectEqual(@as(u8, 0xF8), ctx.timer.readTac() & 0xFC);
}

test "TIMA falling edge bug: resetting DIV" {
    var ctx = try setup();
    defer ctx.arena.deinit();

    // Enable timer, speed 4096Hz (bit 9 watched)
    ctx.timer.writeTac(0x04);
    ctx.timer.writeTima(0);

    // Tick until bit 9 of internal counter is 1
    // Bit 9 flips every 512 T-cycles (128 M-cycles)
    ctx.timer.tick(512);
    try testing.expectEqual(@as(u16, 512), ctx.timer.counter);
    try testing.expectEqual(@as(u1, 1), @as(u1, @truncate(ctx.timer.counter >> 9)));

    // Resetting DIV should flip bit 9 from 1 to 0, triggering TIMA increment
    ctx.timer.writeDiv();
    try testing.expectEqual(@as(u8, 1), ctx.timer.readTima());
}

test "TIMA falling edge bug: disabling TAC" {
    var ctx = try setup();
    defer ctx.arena.deinit();

    // Enable timer, speed 4096Hz
    ctx.timer.writeTac(0x04);
    ctx.timer.writeTima(0);

    // Tick until watched bit (bit 9) is 1
    ctx.timer.tick(512);
    try testing.expectEqual(@as(u1, 1), @as(u1, @truncate(ctx.timer.counter >> 9)));

    // Disabling timer should trigger falling edge if bit was 1
    ctx.timer.writeTac(0x00); // Bit 2 = 0
    try testing.expectEqual(@as(u8, 1), ctx.timer.readTima());
}

test "TIMA increment" {
    var ctx = try setup();
    defer ctx.arena.deinit();

    ctx.timer.writeTac(0x04);
    ctx.timer.tick(1024);

    try testing.expectEqual(@as(u8, 1), ctx.timer.readTima());
}

test "TIMA overflow and TMA reload delay" {
    var ctx = try setup();
    defer ctx.arena.deinit();

    ctx.timer.writeTma(0x55);
    ctx.timer.writeTima(0xFF);
    ctx.timer.writeTac(0x04); // Enable, 4096Hz

    // One more tick to trigger overflow
    // In 4096Hz mode, it ticks every 1024 T-cycles (256 M-cycles)
    // But we can manually tick the hardware or just wait.
    // Let's tick enough to cause the increment.
    ctx.timer.tick(1024);

    // After overflow, TIMA should be 0 for 1 machine cycle (4 T-cycles)
    try testing.expectEqual(@as(u8, 0), ctx.timer.readTima());
    try testing.expect(ctx.timer.pending_interrupt);

    // Tick one more M-cycle to complete the reload
    ctx.timer.tick(4);
    try testing.expectEqual(@as(u8, 0x55), ctx.timer.readTima());
    try testing.expect(ctx.interrupts.isPending(.timer));
}
