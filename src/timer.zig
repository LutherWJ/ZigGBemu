const std = @import("std");
const hw = @import("constants.zig");

pub const Timer = struct {
    counter: u16 = 0, //
    tima: u8 = 0, // Timer counter
    tma: u8 = 0, // timer modulo
    tac: TimerControl = 0,

    pub fn incrementTimer(self: *Timer, cycles: u8) void {
        self.counter += cycles;
    }

    pub fn resetDiv(self: *Timer) void {
        self.counter &= 0xFF;
    }
};

const TimerControl = packed struct {
    clock_select: u2,
    enable: u1,
    padding: u5, // useless
};
