const hw = @import("hw");

pub const Header = struct {
    title: [16]u8,
    cart_type: u8,
    rom_size: u8,
    ram_size: u8, // Just a code for ram size

    pub fn parse(rom: []const u8) Header {
        return .{
            .title = rom[hw.Header.title_start..hw.Header.title_end].*,
            .cart_type = rom[hw.Header.cart_type],
            .rom_size = rom[hw.Header.rom_size],
            .ram_size = rom[hw.Header.ram_size],
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
