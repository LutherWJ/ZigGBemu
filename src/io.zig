const std = @import("std");
const Timer = @import("timer").Timer;
const Joypad = @import("joypad").Joypad;
const hw = @import("hw");
const Interrupts = @import("interrupts").Interrupts;

pub const Io = struct {
    timer: *Timer,
    joypad: *Joypad,
    interrupts: *Interrupts,

    pub fn read(self: *const Io, address: u16) u8 {
        return switch (address) {
            hw.Io.joyp => self.joypad.read(),
            hw.Io.div => self.timer.readDiv(),
            hw.Io.tima => self.timer.readTima(),
            hw.Io.tma => self.timer.readTma(),
            hw.Io.tac => self.timer.readTac(),
            hw.Io.if_reg => self.interrupts.ifr,
            else => {
                std.debug.print("Attempted reading from unimplemented memory region at address: 0x{X}\n", .{address});
                return 0;
            },
        };
    }

    pub fn write(self: *Io, address: u16, value: u8) void {
        switch (address) {
            hw.Io.div => self.timer.writeDiv(),
            hw.Io.tima => self.timer.writeTima(value),
            hw.Io.tma => self.timer.writeTma(value),
            hw.Io.tac => self.timer.writeTac(value),
            hw.Io.if_reg => self.interrupts.ifr = value,
            else => std.debug.print("Attempted writing to unimplemented memory region at address: 0x{X}\n", .{address}),
        }
    }
};
