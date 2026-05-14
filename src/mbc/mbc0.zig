const hw = @import("hw");

// mbc0 means no mbc, direct memory mapping.

pub const Mbc0 = struct {
    rom: []const u8,
    ram: [hw.Mbc.ram_bank_size]u8 = [_]u8{0} ** hw.Mbc.ram_bank_size,

    pub fn read(self: *const Mbc0, address: u16) u8 {
        return switch (address) {
            hw.Map.rom0.start...hw.Map.romx.end => self.rom[address],
            hw.Map.ext_ram.start...hw.Map.ext_ram.end => self.ram[address - hw.Map.ext_ram.start],
            else => 0xFF,
        };
    }

    pub fn write(self: *Mbc0, address: u16, value: u8) void {
        switch (address) {
            hw.Map.ext_ram.start...hw.Map.ext_ram.end => {
                self.ram[address - hw.Map.ext_ram.start] = value;
            },
            else => {},
        }
    }
};
