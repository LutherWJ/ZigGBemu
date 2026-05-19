pub const Sram = struct {
    data: []u8,
    dirty: bool,
    has_battery: bool,

    pub fn read(self: *const Sram, address: usize) u8 {
        if (address >= self.data.len) return 0xFF;
        return self.data[address];
    }

    pub fn write(self: *Sram, address: usize, value: u8) void {
        if (address >= self.data.len) return;
        self.data[address] = value;
        if (self.has_battery) self.dirty = true;
    }
};
