const hw = @import("constants.zig");

// mbc0 means no mbc, direct memory mapping.

pub const MBC0 = struct {
    rom: []const u8,
    ram: [hw.mbc.ram_bank_size]u8 = [_]u8{0} ** hw.mbc.ram_bank_size,

    pub fn read(self: *const MBC0, address: u16) u8 {
        return switch (address) {
            hw.map.rom0.start...hw.map.romx.end => self.rom[address],
            hw.map.ext_ram.start...hw.map.ext_ram.end => self.ram[address - hw.map.ext_ram.start],
            else => 0xFF,
        };
    }

    pub fn write(self: *MBC0, address: u16, value: u8) void {
        switch (address) {
            hw.map.ext_ram.start...hw.map.ext_ram.end => {
                self.ram[address - hw.map.ext_ram.start] = value;
            },
            else => {},
        }
    }
};
