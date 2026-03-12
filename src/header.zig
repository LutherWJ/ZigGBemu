const std = @import("std");

pub const Header = struct {
    title: [16]u8,
    cart_type: u8,
    rom_size: u8,
    ram_size: u8, // Just a code for ram size

    // Takes a rom as a parameter
    pub fn parse(rom: []const u8) Header {
        return .{
            .title = rom[0x0134..0x0144].*,
            .cart_type = rom[0x0147],
            .rom_size = rom[0x0148],
            .ram_size = rom[0x0149],
        };
    }

    pub fn getRamSize(self: *Header) usize {
        return switch (self.ram_size) {
            0x02 => 8 * 1024,
            0x03 => 32 * 1024,
            0x04 => 128 * 1024,
            0x05 => 64 * 1024,
            else => 0,
        };
    }
};
