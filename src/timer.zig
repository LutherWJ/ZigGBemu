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
    tima: u8 = 0,
    tma: u8 = 0, // timer modulo
    tac: TimerControl = .{},
    pending_interrupt: bool = false, // Interupt is delayed one machine cycle
    interrupts: *Interrupts,

    pub fn writeDiv(self: *Timer) void {
        const watched_bit = self.getWatchedBit();
        const old_mux: u1 = @truncate(self.counter >> watched_bit);

        self.counter = 0;

        const new_mux: u1 = @truncate(self.counter >> watched_bit);
        if (old_mux == 1 and new_mux == 0) {
            self.incrementTimer();
        }
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

        // Check for fallin edge bug conditions
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
        return @truncate(self.counter >> 8);
    }

    pub fn readTma(self: *const Timer) u8 {
        return self.tma;
    }

    pub fn readTac(self: *const Timer) u8 {
        return @as(u8, @bitCast(self.tac)) | 0b11111000;
    }

    // Takes T-cycles as an argument
    pub fn tick(self: *Timer, cycles: u32) void {
        // Function heavily assumes that cycles will always be a multiple of 4.
        std.debug.assert(cycles % hw.Timings.tCyclesPerMCycle == 0);

        var cycles_left = cycles;

        // Extract T cycles into machine cycles
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

    // Ticks the counter one machine cycle
    fn tickTimerHardware(self: *Timer) void {
        if (self.pending_interrupt) {
            self.tima = self.tma;
            self.interrupts.request(.timer);
            self.pending_interrupt = false;
        }

        const watched_bit = self.getWatchedBit();
        const before = (self.counter >> watched_bit) & 1;

        self.counter +%= @as(u16, hw.Timings.tCyclesPerMCycle);

        const after = (self.counter >> watched_bit) & 1;

        if (self.tac.enable == 1 and before == 1 and after == 0) {
            self.incrementTimer();
        }
    }

    fn incrementTimer(self: *Timer) void {
        const result = @addWithOverflow(self.tima, 1);
        self.tima = result[0];
        const overflow: u1 = result[1];

        if (overflow == 1) {
            self.pending_interrupt = true;
            self.tima = 0;
        }
    }
};
