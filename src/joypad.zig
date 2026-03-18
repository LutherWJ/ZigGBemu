pub const Joypad = struct {
    a: bool = false,
    b: bool = false,
    select: bool = false,
    start: bool = false,
    right: bool = false,
    left: bool = false,
    up: bool = false,
    down: bool = false,

    pub fn read(self: *const Joypad) u8 {
        _ = self;
        return 0;
    }
};
