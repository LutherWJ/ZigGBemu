const std = @import("std");
const testing = std.testing;
const Mbc1 = @import("mbc1").Mbc1;
const hw = @import("constants");

fn setup_test_rom() [128 * hw.Mbc.rom_bank_size]u8 {
    var rom = [_]u8{0} ** (128 * hw.Mbc.rom_bank_size);
    // Initialize first byte of each bank to its bank number to easily identify banks
    for (0..128) |bank| {
        rom[bank * hw.Mbc.rom_bank_size] = @intCast(bank);
    }
    return rom;
}

test "MBC1: RAM Enable/Disable" {
    std.debug.print("Running MBC1: RAM Enable/Disable...\n", .{});
    var rom = setup_test_rom();
    var ram = [_]u8{0} ** hw.Mbc.ram_bank_size;
    var mbc1 = Mbc1{ .rom = &rom, .ram = &ram };

    // RAM disabled by default
    try testing.expectEqual(@as(u8, 0xFF), mbc1.read(0xA000));

    // Enable RAM
    mbc1.write(0x0000, 0x0A);
    
    // Default RAM is initialized to 0
    try testing.expectEqual(@as(u8, 0x00), mbc1.read(0xA000));

    // Write to RAM and verify
    mbc1.write(0xA000, 0x42);
    try testing.expectEqual(@as(u8, 0x42), mbc1.read(0xA000));

    // Disable RAM
    mbc1.write(0x0000, 0x00);
    try testing.expectEqual(@as(u8, 0xFF), mbc1.read(0xA000));
}

test "MBC1: ROM Banking (Upper Region)" {
    std.debug.print("Running MBC1: ROM Banking (Upper Region)...\n", .{});
    var rom = setup_test_rom();
    var ram = [_]u8{0} ** hw.Mbc.ram_bank_size;
    var mbc1 = Mbc1{ .rom = &rom, .ram = &ram };

    // Default Bank 1
    try testing.expectEqual(@as(u8, 0x01), mbc1.read(0x4000));

    // Select Bank 5
    mbc1.write(0x2000, 0x05);
    try testing.expectEqual(@as(u8, 0x05), mbc1.read(0x4000));

    // Bank 0 translates to Bank 1
    mbc1.write(0x2000, 0x00);
    try testing.expectEqual(@as(u8, 0x01), mbc1.read(0x4000));

    // Select Bank 0x20 (translates to 0x21 due to bank 0 translation)
    mbc1.write(0x4000, 0x01); // secondary bank register = 1
    mbc1.write(0x2000, 0x00); // primary bank register = 0
    try testing.expectEqual(@as(u8, 0x21), mbc1.read(0x4000));
    
    // Select Bank 0x25
    mbc1.write(0x4000, 0x01); // secondary = 1
    mbc1.write(0x2000, 0x05); // primary = 5
    try testing.expectEqual(@as(u8, 0x25), mbc1.read(0x4000));
}

test "MBC1: Bit Masking (Out of bounds ROM)" {
    std.debug.print("Running MBC1: Bit Masking (Out of bounds ROM)...\n", .{});
    var rom = [_]u8{0} ** (4 * hw.Mbc.rom_bank_size); // 4 banks
    for (0..4) |bank| {
        rom[bank * hw.Mbc.rom_bank_size] = @intCast(bank);
    }
    var ram = [_]u8{0} ** hw.Mbc.ram_bank_size;
    var mbc1 = Mbc1{ .rom = &rom, .ram = &ram };

    // Request bank 7 (binary 111). Mask should be 3 (binary 011), so it maps to bank 3.
    mbc1.write(0x2000, 0x07);
    try testing.expectEqual(@as(u8, 0x03), mbc1.read(0x4000));
}

test "MBC1: Mode 0 (ROM Banking Mode)" {
    std.debug.print("Running MBC1: Mode 0 (ROM Banking Mode)...\n", .{});
    var rom = setup_test_rom();
    var ram = [_]u8{0} ** (4 * hw.Mbc.ram_bank_size);
    for (0..4) |bank| {
        ram[bank * hw.Mbc.ram_bank_size] = @intCast(bank);
    }
    var mbc1 = Mbc1{ .rom = &rom, .ram = &ram };
    mbc1.write(0x0000, 0x0A); // Enable RAM

    // Ensure we are in Mode 0 (default)
    mbc1.write(0x6000, 0x00);

    // Set secondary bank register
    mbc1.write(0x4000, 0x01);

    // Lower ROM always targets Bank 0 in Mode 0
    try testing.expectEqual(@as(u8, 0x00), mbc1.read(0x0000));

    // RAM always targets Bank 0 in Mode 0
    try testing.expectEqual(@as(u8, 0x00), mbc1.read(0xA000));
}

test "MBC1: Mode 1 (RAM / Advanced ROM Mode)" {
    std.debug.print("Running MBC1: Mode 1 (RAM / Advanced ROM Mode)...\n", .{});
    var rom = setup_test_rom();
    var ram = [_]u8{0} ** (4 * hw.Mbc.ram_bank_size);
    for (0..4) |bank| {
        ram[bank * hw.Mbc.ram_bank_size] = @intCast(bank);
    }
    var mbc1 = Mbc1{ .rom = &rom, .ram = &ram };
    mbc1.write(0x0000, 0x0A); // Enable RAM

    // Enter Mode 1
    mbc1.write(0x6000, 0x01);

    // Set secondary bank register to 1
    mbc1.write(0x4000, 0x01);

    // Advanced Lower ROM targets Bank 0x20 in Mode 1 with secondary = 1
    try testing.expectEqual(@as(u8, 0x20), mbc1.read(0x0000));

    // RAM targets Bank 1 in Mode 1 with secondary = 1
    try testing.expectEqual(@as(u8, 0x01), mbc1.read(0xA000));
}

test "MBC1: Integration - Mode switching on the fly" {
    std.debug.print("Running MBC1: Integration - Mode switching on the fly...\n", .{});
    var rom = setup_test_rom();
    var ram = [_]u8{0} ** (4 * hw.Mbc.ram_bank_size);
    var mbc1 = Mbc1{ .rom = &rom, .ram = &ram };
    
    mbc1.write(0x0000, 0x0A); // Enable RAM
    mbc1.write(0x4000, 0x01); // Secondary Bank = 1

    // Write 0x55 to RAM Bank 0
    mbc1.write(0x6000, 0x00); // Enter Mode 0
    mbc1.write(0xA000, 0x55);
    
    // Enter Mode 1
    mbc1.write(0x6000, 0x01);
    
    // Write 0x99 to RAM Bank 1, Address 0xA000
    // Because we are in Mode 1 and Secondary Bank = 1, address 0xA000 writes to Bank 1
    mbc1.write(0xA000, 0x99);

    // Verify Mode 1 reads Bank 1
    try testing.expectEqual(@as(u8, 0x99), mbc1.read(0xA000));

    // Switch back to Mode 0
    mbc1.write(0x6000, 0x00);

    // Verify Mode 0 reads Bank 0
    try testing.expectEqual(@as(u8, 0x55), mbc1.read(0xA000));
}
