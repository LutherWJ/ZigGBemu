pub const Joypad = packed struct(u8) {
    a_right: u1 = 1,
    b_left: u1 = 1,
    select_up: u1 = 1,
    start_down: u1 = 1,
    select_dpad: u1 = 1,
    select_buttons: u1 = 1,
    padding: u2 = 3,

    pub fn read(self: *const Joypad) u8 {
        return @bitCast(self.*);
    }

    pub fn write(self: *Joypad, value: u8) void {
        self.* = @bitCast((@as(u8, @bitCast(self.*)) & 0x0F) | (value & 0xF0));
    }
};
