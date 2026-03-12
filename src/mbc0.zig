// mbc0 means no mbc, direct memory mapping.

pub const MBC0 = struct {
    rom: []const u8,
    ram: []u8,

    pub fn read(self: MBC0, address: u16) u8 {
        return switch (address) {
            0x0000...0x7FFF => self.rom[address],
            0xA000...0xBFFF => if (self.ram.len > 0) self.ram[address - 0xA000] else 0xFF,
            else => 0xFF,
        };
    }

    pub fn write(self: *MBC0, address: u16, value: u8) void {
        switch (address) {
            0xA000...0xBFFF => if (self.ram.len > 0) {
                self.ram[address - 0xA000] = value;
            },
            else => {},
        }
    }
};
