pub const Button = enum { a, b, up, down, left, right, select, start };

pub const Joypad = struct {
    a: bool = false,
    b: bool = false,
    up: bool = false,
    down: bool = false,
    left: bool = false,
    right: bool = false,
    start: bool = false,
    select: bool = false,

    select_dpad: u1 = 1,
    select_buttons: u1 = 1,

    pub fn setButton(self: *Joypad, btn: Button, pressed: bool) void {
        switch (btn) {
            .a => self.a = pressed,
            .b => self.b = pressed,
            .up => self.up = pressed,
            .down => self.down = pressed,
            .left => self.left = pressed,
            .right => self.right = pressed,
            .select => self.select = pressed,
            .start => self.start = pressed,
        }
    }

    pub fn read(self: *const Joypad) u8 {
        var input_bits: u4 = 0b1111;

        if (self.select_dpad == 0) {
            if (self.right) input_bits &= ~(@as(u4, 1) << 0);
            if (self.left) input_bits &= ~(@as(u4, 1) << 1);
            if (self.up) input_bits &= ~(@as(u4, 1) << 2);
            if (self.down) input_bits &= ~(@as(u4, 1) << 3);
        }

        if (self.select_buttons == 0) {
            if (self.a) input_bits &= ~(@as(u4, 1) << 0);
            if (self.b) input_bits &= ~(@as(u4, 1) << 1);
            if (self.select) input_bits &= ~(@as(u4, 1) << 2);
            if (self.start) input_bits &= ~(@as(u4, 1) << 3);
        }

        // Reconstruct the 8-bit register
        // Top 2 bits are unused (always 1), bits 4/5 are the selectors, bits 0-3 are inputs
        return 0xC0 | (@as(u8, self.select_buttons) << 5) | (@as(u8, self.select_dpad) << 4) | input_bits;
    }

    pub fn write(self: *Joypad, value: u8) void {
        self.select_dpad = @truncate(value >> 4);
        self.select_buttons = @truncate(value >> 5);
    }
};
