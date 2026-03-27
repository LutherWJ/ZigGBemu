const std = @import("std");
const Interrupts = @import("interrupts").Interrupts;

const LinearFifo = std.fifo.LinearFifo;
const LcdStatus = packed struct(u8) {
    ppu_mode: u2 = 0,
    ly_eql_lyc: u1 = 0,
    mode0_sel: u1 = 0,
    mode1_sel: u1 = 0,
    mode2_sel: u1 = 0,
    lyc_sel: u1 = 0,
    pad: u1 = 0,
};

const LcdControl = packed struct(u8) {
    bg_win_enable: u1 = 0,
    obj_enable: u1 = 0,
    obj_size: u1 = 0,
    bg_tile_map: u1 = 0,
    bg_win_tiles: u1 = 0,
    win_enable: u1 = 0,
    win_tile_map: u1 = 0,
    lcd_enable: u1 = 0,
};

const PpuStates = enum {};

pub const Ppu = struct {
    lcdc: LcdControl = 0, // control register
    stat: LcdStatus = 0, // status register
    scy: u8 = 0, // vertical scroll register
    scx: u8 = 0, // horizontal scroll register
    ly: u8 = 0, // scanline register
    lyc: u8 = 0, // scanline control register

    _bg_buf: [32]u8 = [_]u8{0} ** 32,
    _obj_buf: [32]u8 = [_]u8{0} ** 32,
    bg_fifo: LinearFifo(u8, .Static),
    obj_fifo: LinearFifo(u8, .Static),

    interrupts: *Interrupts,

    pub fn init(self: *Ppu, interrupts: *Interrupts) void {
        self.* = Ppu{
            .interrupts = interrupts,
            .bg_fifo = undefined,
            .obj_fifo = undefined,
        };

        self.bg_fifo = LinearFifo(u8, .Static).initBuffer(&self._bg_buf);
        self.obj_fifo = LinearFifo(u8, .Static).initBuffer(&self._obj_buf);
    }

    pub fn tick(self: *Ppu) void {
        _ = self;
    }
};
