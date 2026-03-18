const std = @import("std");
const hw = @import("hw");
const interrupts = @import("interrupts");
const Interrupts = interrupts.Interrupts;

const TimerControl = packed struct(u8) {
    clock_select: u2 = 0,
    enable: u1 = 0,
    unused: u5 = 0x1F,
};

pub const Timer = struct {
    counter: u16 = 0, // Counts how many Hz have passed since last timer increment
    div: u8 = 0,
    tima: u8 = 0,
    tma: u8 = 0, // timer modulo
    tac: TimerControl = .{},
    pending_interrupt: bool = false, // Interupt is delayed one machine cycle
    interrupts: *Interrupts,

    pub fn writeDiv(self: *Timer) void {
        // TODO: Implement falling edge bug
        self.div = 0;
        self.tima = 0;
    }

    pub fn writeTima(self: *Timer, value: u8) void {
        self.tima = value;
    }

    pub fn writeTac(self: *Timer, value: u8) void {
        // Record old state before mutation
        const old_watched_bit = self.getWatchedBit();
        const old_enable = self.tac.enable;
        const old_mux: u1 = @truncate(self.counter >> old_watched_bit);
        const old = old_enable & old_mux;

        self.tac.enable = @truncate(value >> 2);
        self.tac.clock_select = @truncate(value);

        const new_watched_bit = self.getWatchedBit();
        const new_mux = (self.counter >> new_watched_bit) & 1;
        const new = self.tac.enable & new_mux;

        // Check for falling edge bug conditions
        if (old == 1 and new == 0) {
            self.incrementTimer();
        }
    }

    pub fn writeTma(self: *Timer, value: u8) void {
        self.tma = value;
    }

    pub fn readTima(self: *const Timer) u8 {
        return self.tima;
    }

    pub fn readDiv(self: *const Timer) u8 {
        return self.div;
    }

    pub fn readTma(self: *const Timer) u8 {
        return self.tma;
    }

    pub fn readTac(self: *const Timer) u8 {
        return @bitCast(self.tac);
    }

    pub fn tick(self: *Timer, cycles: u8) void {
        std.debug.assert(cycles % hw.Timings.tCyclesPerMCycle == 0);

        var cycles_left = cycles;
        while (cycles_left > 0) : (cycles_left -= hw.Timings.tCyclesPerMCycle) {
            self.tickTimerHardware();
        }
    }

    fn getWatchedBit(self: *const Timer) u4 {
        return switch (self.tac.clock_select) {
            0b00 => 9,
            0b01 => 3,
            0b10 => 5,
            0b11 => 7,
        };
    }

    fn tickTimerHardware(self: *Timer) void {
        const watched_bit = self.getWatchedBit();
        const before = (self.counter >> watched_bit) & 1;

        self.counter +%= @as(u16, hw.Timings.tCyclesPerMCycle);

        const after = (self.counter >> watched_bit) & 1;

        if (self.tac.enable == 1 and before == 1 and after == 0) {
            self.incrementTimer();
        }
    }

    fn incrementTimer(self: *Timer) void {
        if (self.pending_interrupt) {
            self.tima = self.tma;
            self.pending_interrupt = false;
            self.interrupts.request(.timer);
            return;
        }

        const result = @addWithOverflow(self.tima, 1);
        self.tima = result[0];
        const overflow: u1 = result[1];

        if (overflow == 1) {
            self.div += 1;
            self.pending_interrupt = true;
            self.tima = 0;
        }
    }
};
