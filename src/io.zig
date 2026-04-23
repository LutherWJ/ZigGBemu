const std = @import("std");
const Timer = @import("timer").Timer;
const Joypad = @import("joypad").Joypad;
const hw = @import("hw");
const Interrupts = @import("interrupts").Interrupts;
const Sdt = @import("sdt").Sdt;
const Ppu = @import("ppu").Ppu;

pub const Io = struct {
    timer: *Timer,
    interrupts: *Interrupts,
    joypad: *Joypad,
    sdt: *Sdt,
    ppu: *Ppu,

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
            hw.Io.lcdc => @bitCast(self.ppu.lcdc),
            hw.Io.stat => @bitCast(self.ppu.stat),
            hw.Io.scy => self.ppu.scy,
            hw.Io.scx => self.ppu.scx,
            hw.Io.ly => self.ppu.ly,
            hw.Io.lyc => self.ppu.lyc,
            hw.Io.bgp => self.ppu.bgp,
            hw.Io.obp0 => self.ppu.obp0,
            hw.Io.obp1 => self.ppu.obp1,
            hw.Io.wy => self.ppu.wy,
            hw.Io.wx => self.ppu.wx,
            hw.Io.dma => self.ppu.dma,
            else => {
                // std.debug.print("Attempted reading from unimplemented IO region at address: 0x{X}\n", .{address});
                return 0xFF;
            },
        };
    }

    pub fn write(self: *Io, address: u16, value: u8) void {
        switch (address) {
            hw.Io.joyp => self.joypad.write(value),
            hw.Io.sb => self.sdt.writeSb(value),
            hw.Io.sc => self.sdt.writeSc(value),
            hw.Io.div => self.timer.writeDiv(),
            hw.Io.tima => self.timer.writeTima(value),
            hw.Io.tma => self.timer.writeTma(value),
            hw.Io.tac => self.timer.writeTac(value),
            hw.Io.if_reg => self.interrupts.ifr = value,
            hw.Io.lcdc => self.ppu.lcdc = @bitCast(value),
            hw.Io.stat => self.ppu.stat = @bitCast(value),
            hw.Io.scy => self.ppu.scy = value,
            hw.Io.scx => self.ppu.scx = value,
            hw.Io.ly => self.ppu.ly = value,
            hw.Io.lyc => self.ppu.lyc = value,
            hw.Io.dma => self.ppu.writeDma(value),
            hw.Io.bgp => self.ppu.bgp = value,
            hw.Io.obp0 => self.ppu.obp0 = value,
            hw.Io.obp1 => self.ppu.obp1 = value,
            hw.Io.wy => self.ppu.wy = value,
            hw.Io.wx => self.ppu.wx = value,
            else => {
                // if (@import("builtin").os.tag != .freestanding) {
                //     std.debug.print("[IO] Write to unimplemented address: 0x{X:0>4} = 0x{X:0>2}\n", .{ address, value });
                // }
            },
        }
    }
};
