// TODO: I have literally zero clue how this is supposed to work, making it a stub for now.
// This probably needs to wait until I have the interface conventions clearly defined.
const std = @import("std");

const SerialTransferControl = packed struct(u8) {
    clock_select: u1 = 0, // 1 = enable, 0 = disable
    clock_speed: u1 = 0, // useless on original gameboy
    _padding: u5 = 0, // useless
    enable: u1 = 0, // 1 = master, 0 = slave
};

// Serial Data Transfer
pub const Sdt = struct {
    sb: u8 = 0,
    sc: SerialTransferControl = .{},

    pub fn writeSc(self: *Sdt, value: u8) void {
        self.sc = @bitCast(value);

        if (value == 0x81) {
            std.debug.print("{c}", .{self.sb});
            self.sc.enable = 0;
        }
    }

    pub fn writeSb(self: *Sdt, value: u8) void {
        self.sb = value;
    }

    pub fn tick(self: *Sdt) void {
        if (self.sc.clock_select == 0 or self.sc.enable == 0) return;
    }
};
