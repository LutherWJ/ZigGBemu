const std = @import("std");
const builtin = @import("builtin");
const interrupts = @import("interrupts");
const Interrupts = interrupts.Interrupts;
const LinearFifo = std.fifo.LinearFifo;

const SerialTransferControl = packed struct(u8) {
    clock_select: u1 = 0, // 0 = External, 1 = Internal
    clock_speed: u1 = 0, // CGB only
    _padding: u5 = 0,
    enable: u1 = 0, // 0 = No transfer, 1 = Start
};

// Serial Data Transfer
pub const Sdt = struct {
    sb: u8 = 0,
    sc: SerialTransferControl = .{},
    interrupts: *Interrupts,
    cycles_left: u16 = 0,

    // Ring buffer for serial output
    // gone for now.

    pub fn writeSc(self: *Sdt, value: u8) void {
        self.sc = @bitCast(value);
        if (self.sc.enable == 1 and self.sc.clock_select == 1) {
            self.cycles_left = 128;
        }
    }

    pub fn writeSb(self: *Sdt, value: u8) void {
        self.sb = value;
    }

    pub fn tick(self: *Sdt) void {
        if (self.sc.enable == 1 and self.sc.clock_select == 1) {
            if (self.cycles_left > 0) {
                self.cycles_left -= 1;
                if (self.cycles_left == 0) {
                    // self.fifo.writeItem(self.sb) catch {};
                    if (builtin.os.tag != .freestanding) {
                        std.debug.print("{c}", .{self.sb});
                    }
                    self.sc.enable = 0;
                    self.interrupts.request(.serial);
                }
            }
        }
    }
};
