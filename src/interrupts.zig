const std = @import("std");

pub const InterruptBit = enum(u3) {
    vblank = 0,
    lcd = 1,
    timer = 2,
    serial = 3,
    joypad = 4,
};

pub const Interrupts = struct {
    ie: u8 = 0,
    ifr: u8 = 0,

    pub fn request(self: *Interrupts, interrupt: InterruptBit) void {
        self.ifr |= (@as(u8, 1) << @intFromEnum(interrupt));
    }

    pub fn acknowledge(self: *Interrupts, interrupt: InterruptBit) void {
        self.ifr &= ~(@as(u8, 1) << @intFromEnum(interrupt));
    }

    pub fn isPending(self: *const Interrupts, interrupt: InterruptBit) bool {
        const bit = @as(u8, 1) << @intFromEnum(interrupt);
        return (self.ie & bit) != 0 and (self.ifr & bit) != 0;
    }

    pub fn getPending(self: *const Interrupts) ?InterruptBit {
        const pending = self.ie & self.ifr & 0x1F;
        if (pending == 0) return null;
        // Find and return the first set bit
        return @enumFromInt(@as(u3, @truncate(@ctz(pending))));
    }
};
