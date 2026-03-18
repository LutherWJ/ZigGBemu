const std = @import("std");
const testing = std.testing;
const Mbc0 = @import("mbc0").Mbc0;
const hw = @import("hw");

test "MBC0: ROM read" {
    std.debug.print("Running MBC0: ROM read...\n", .{});
    var rom = [_]u8{0} ** 0x8000;
    rom[0x0000] = 0x11;
    rom[0x3FFF] = 0x22;
    rom[0x4000] = 0x33;
    rom[0x7FFF] = 0x44;

    const mbc0 = Mbc0{ .rom = &rom };

    try testing.expectEqual(@as(u8, 0x11), mbc0.read(0x0000));
    try testing.expectEqual(@as(u8, 0x22), mbc0.read(0x3FFF));
    try testing.expectEqual(@as(u8, 0x33), mbc0.read(0x4000));
    try testing.expectEqual(@as(u8, 0x44), mbc0.read(0x7FFF));
}

test "MBC0: RAM read/write" {
    std.debug.print("Running MBC0: RAM read/write...\n", .{});
    const rom = [_]u8{0} ** 0x8000;
    var mbc0 = Mbc0{ .rom = &rom };

    // Default RAM should be 0
    try testing.expectEqual(@as(u8, 0), mbc0.read(0xA000));
    try testing.expectEqual(@as(u8, 0), mbc0.read(0xBFFF));

    // Write to RAM
    mbc0.write(0xA000, 0x55);
    mbc0.write(0xBFFF, 0xAA);

    try testing.expectEqual(@as(u8, 0x55), mbc0.read(0xA000));
    try testing.expectEqual(@as(u8, 0xAA), mbc0.read(0xBFFF));
}

test "MBC0: Write to ROM should be ignored" {
    std.debug.print("Running MBC0: Write to ROM should be ignored...\n", .{});
    var rom = [_]u8{0} ** 0x8000;
    rom[0x1234] = 0x55;
    var mbc0 = Mbc0{ .rom = &rom };

    mbc0.write(0x1234, 0xAA);
    try testing.expectEqual(@as(u8, 0x55), mbc0.read(0x1234));
}

test "MBC0: Read out of bounds" {
    std.debug.print("Running MBC0: Read out of bounds...\n", .{});
    const rom = [_]u8{0} ** 0x8000;
    const mbc0 = Mbc0{ .rom = &rom };

    // These are not handled by MBC0 typically but let's check current implementation
    // mbc0.read returns 0xFF for unhandled addresses
    try testing.expectEqual(@as(u8, 0xFF), mbc0.read(0x8000));
    try testing.expectEqual(@as(u8, 0xFF), mbc0.read(0x9FFF));
    try testing.expectEqual(@as(u8, 0xFF), mbc0.read(0xC000));
}
