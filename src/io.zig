const std = @import("std");
const Timer = @import("timer").Timer;
const Joypad = @import("joypad").Joypad;
const hw = @import("hw");
const Interrupts = @import("interrupts").Interrupts;
const Sdt = @import("sdt").Sdt;

pub const Io = struct {
    timer: *Timer,
    interrupts: *Interrupts,
    joypad: *Joypad,
    sdt: *Sdt,

    pub fn read(self: *const Io, address: u16) u8 {
        return switch (address) {
            hw.Io.joyp => self.joypad.read(),
            hw.Io.sb => self.sdt.sb,
            hw.Io.sc => @bitCast(self.sdt.sc),
            hw.Io.div => self.timer.readDiv(),
            hw.Io.tima => self.timer.readTima(),
            hw.Io.tma => self.timer.readTma(),
            hw.Io.tac => self.timer.readTac(),
            hw.Io.if_reg => self.interrupts.ifr,
            hw.Io.ly => 144, // Stub for testing
            else => {
                // std.debug.print("Attempted reading from unimplemented IO region at address: 0x{X}\n", .{address});
                return 0;
            },
        };
    }

    pub fn write(self: *Io, address: u16, value: u8) void {
        switch (address) {
            hw.Io.joyp => self.joypad.write(value),
            hw.Io.sb => self.sdt.writeSb(value), // ???
            hw.Io.sc => self.sdt.writeSc(value),
            hw.Io.div => self.timer.writeDiv(),
            hw.Io.tima => self.timer.writeTima(value),
            hw.Io.tma => self.timer.writeTma(value),
            hw.Io.tac => self.timer.writeTac(value),
            hw.Io.if_reg => self.interrupts.ifr = value,
            else => std.debug.print("[IO] Write to unimplemented address: 0x{X:0>4} = 0x{X:0>2}\n", .{address, value}),
        }
    }
};
