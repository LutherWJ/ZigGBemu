const std = @import("std");

// NR10
const Audsweep = packed struct(u8) {
    individual_step: u3 = 0,
    direction: u1 = 0,
    pace: u3 = 0,
    padding: u1 = 0,
};

// NR11, NR21
const Audlen = packed struct(u8) {
    initial_length_timer: u6 = 0,
    wave_duty: u2 = 0,
};

// NR12, NR24
const Audenv = packed struct(u8) {
    sweep_pace: u3 = 0,
    env_dir: u1 = 0,
    initial_volume: u4 = 0,
};

// NR13/NR23/NR33 (Audlow) Stores the lower 8 bits of the 11-bit period value and can be represented by a simple u8.

// NR14, NR24
const Audhigh = packed struct(u8) {
    period: u3 = 0,
    pad: u3 = 0,
    length_enable: u1 = 0,
    trigger: u1 = 0, // write only.
};

// NR30
const Aud3ena = packed struct(u8) {
    padding: u7 = 0,
    dac_enable: u1 = 0,
};

// NR31 (Aud3len) stores the intiial length timer and can be represented by a simple u8

// NR32
const Aud3level = packed struct(u8) {
    padding: u5 = 0,
    output_level: u2 = 0,
    padding_bit: u1 = 0,
};

// NR41
const Aud4len = packed struct(u8) {
    initial_length_timer: u6 = 0,
    padding: u2 = 0,
};

// NR43
const Aud4poly = packed struct(u8) {
    clock_divider: u3 = 0,
    lfsr_width: u1 = 0,
    clock_shift: u4 = 0,
};

// NR44
const Aud4go = packed struct(u8) {
    padding: u6 = 0,
    length_enable: u1 = 0,
    trigger: u1 = 0,
};

// NR50
const Audvol = packed struct(u8) {
    right_volume: u3 = 0,
    vin_right: u1 = 0,
    left_volume: u3 = 0,
    vin_left: u1 = 0,
};

// NR51
const Audterm = packed struct(u8) {
    ch1_right: bool = false,
    ch2_right: bool = false,
    ch3_right: bool = false,
    ch4_right: bool = false,
    ch1_left: bool = false,
    ch2_left: bool = false,
    ch3_left: bool = false,
    ch4_left: bool = false,
};

// NR52
const Audena = packed struct(u8) {
    ch1_enable: bool = false,
    ch2_enable: bool = false,
    ch3_enable: bool = false,
    ch4_enable: bool = false,
    padding: u3 = 0,
    audio_enable: bool = false,
};

pub const Apu = struct {
    wave_ram: [16]u8 = [_]u8{0} ** 16,

    // Global control registers
    audterm: Audterm = .{},
    audvol: Audvol = .{},
    audena: Audena = .{},

    // Ch1
    aud1sweep: Audsweep = .{},
    aud1len: Audlen = .{},
    aud1env: Audenv = .{},
    aud1low: u8 = 0,
    aud1high: Audhigh = .{},

    // Ch2
    aud2len: Audlen = .{},
    aud2env: Audenv = .{},
    aud2low: u8 = 0,
    aud2high: Audhigh = .{},

    // Ch3
    aud3ena: Aud3ena = .{},
    aud3len: u8 = 0,
    aud3level: Aud3level = .{},
    aud3low: u8 = 0,
    aud3high: Audhigh = .{},

    // Ch4
    aud4len: Aud4len = .{},
    aud4env: Audenv = .{},
    aud4poly: Aud4poly = .{},
    aud4go: Aud4go = .{},

    pub fn writeAudterm(self: *Apu, value: u8) void {
        self.audterm = @bitCast(value);
    }

    pub fn writeAudvol(self: *Apu, value: u8) void {
        self.audvol = @bitCast(value);
    }

    pub fn writeAudena(self: *Apu, value: u8) void {
        self.audena = @bitCast(value);
    }

    pub fn tick(self: *Apu) void {
        if (!self.audena.audio_enable) return;
    }
};
