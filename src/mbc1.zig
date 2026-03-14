const hw = @import("constants.zig");

pub const MBC1 = struct {
    rom: []const u8,
    ram: []u8,

    // All registers are initialized 0
    ram_enabled: bool = false,
    bank_register: u5 = 0,
    secondary_bank_register: u2 = 0,
    rom_bank_mode: bool = true, // A value of zero means that bank mode is active

    pub fn read(self: *const MBC1, address: u16) u8 {
        return switch (address) {
            hw.map.rom0.start...hw.map.rom0.end => self.readLower(address),
            hw.map.romx.start...hw.map.romx.end => self.readUpper(address),
            hw.map.ext_ram.start...hw.map.ext_ram.end => self.readRam(address),
            else => 0xFF,
        };
    }

    pub fn write(self: *MBC1, address: u16, value: u8) void {
        switch (address) {
            0x0...0x1FFF => self.ramEnableWrite(value),
            0x2000...0x3FFF => self.bank_register = @truncate(value),
            0x4000...0x5FFF => self.secondary_bank_register = @truncate(value),
            0x6000...0x7FFF => self.writeBankMode(value),
            else => {},
        }
    }

    fn readLower(self: *const MBC1, address: u16) u8 {
        if (self.rom_bank_mode) {
            return self.rom[address];
        } else {
            const selected_bank: u8 = @as(u8, self.secondary_bank_register) << 5;
            const num_banks = self.rom.len >> 14;

            const mask: u8 = @truncate(num_banks - 1);
            const safe_bank = selected_bank & mask;

            const offset = @as(usize, safe_bank) * hw.mbc.rom_bank_size;
            const physical_address = @as(usize, address) + offset;
            return self.rom[physical_address];
        }
    }

    // TODO: code is probably fine but look it over again for more optimizations.
    fn readUpper(self: *const MBC1, address: u16) u8 {
        // Reads from the adjustable bank region cannot read from bank 0
        var selected_bank: u8 = (@as(u8, self.secondary_bank_register) << 5) | self.bank_register;
        if (self.bank_register == 0) selected_bank += 1;

        // Standard behavior to avoid out of bounds access is a simple bit truncation
        const num_banks = self.rom.len >> 14;
        const mask: u8 = @truncate(num_banks - 1);
        const safe_bank = selected_bank & mask;

        const offset = @as(usize, safe_bank) * hw.mbc.rom_bank_size;
        const physical_address: usize = @as(usize, (address - hw.mbc.rom_bank_size)) + offset;
        return self.rom[physical_address];
    }

    fn readRam(self: *const MBC1, address: u16) u8 {
        if (!self.ram_enabled) return 0xFF;

        if (self.rom_bank_mode) {
            return self.ram[address - hw.map.ext_ram.start];
        } else {
            const selected_bank = @as(u8, self.secondary_bank_register);
            const num_banks = self.ram.len >> 13;

            const mask: u8 = @truncate(num_banks - 1);
            const safe_bank = selected_bank & mask;

            const physical_address: usize = @as(usize, address - hw.map.ext_ram.start) + (@as(usize, safe_bank) * hw.mbc.ram_bank_size);
            return self.ram[physical_address];
        }
    }

    fn ramEnableWrite(self: *MBC1, value: u8) void {
        const nib: u4 = @truncate(value);
        self.ram_enabled = (nib == 0xA);
    }

    fn writeBankMode(self: *MBC1, value: u8) void {
        const val: u3 = @truncate(value);
        if (val == 0) self.rom_bank_mode = true else self.rom_bank_mode = false;
    }
};
