pub const MBC1 = struct {
    rom: []const u8,
    ram: []u8,

    // All registers are initialized 0
    ram_enabled: bool = false,
    bank_register: u5 = 0,
    secondary_bank_register: u2 = 0,
    is_bank_mode: bool = true, // A value of zero means that bank mode is active

    pub fn read(self: MBC1, address: u16) u8 {
        return switch (address) {
            0x0...0x3FFF => self.rom[address],
            0x4000...0x7FFF => self.readBank(address),
            0xA000...0xBFFF => if (self.ram_enabled) self.ram[address - 0xA000] else 0xFF,
            else => error.undefined, // unimplemented
        };
    }

    pub fn write(self: *MBC1, address: u16, value: u8) void {
        switch (address) {
            0x0...0x1FFF => self.ramEnableWrite(value),
            0x2000...0x3FFF => self.bank_register = @truncate(value),
            0x4000...0x5FFF => self.secondary_bank_register = @truncate(value),
            0x6000...0x7FFF => self.writeBankMode(value),
            else => error.undefined, // unimplemented
        }
    }

    fn readBank(self: *MBC1, address: u16) u8 {
        if (self.selected_bank == 0) return self.rom[address];

        var selected_bank: u8 = 0;
        if (self.is_bank_mode) {
            selected_bank = @as(u8, self.bank_register + (self.secondary_bank_register << 5));
        } else {
            selected_bank = self.bank_register;
        }

        const offset = selected_bank * 0x4000;
        return self.rom[address + offset];
    }

    fn ramEnableWrite(self: *MBC1, value: u8) void {
        const nib: u4 = @truncate(value);
        if (nib == 0xA) self.ram_enabled = true else self.ram_enabled = false;
    }

    fn writeBankMode(self: *MBC1, value: u8) void {
        const val: u3 = @truncate(value);
        if (val == 0) self.is_bank_mode = true else self.is_bank_mode = false;
    }
};
