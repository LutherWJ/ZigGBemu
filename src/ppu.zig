const std = @import("std");
const Interrupts = @import("interrupts").Interrupts;
const hw = @import("hw");
const writeInt = std.mem.writeInt;
const readInt = std.mem.readInt;

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

const ObjectAttributes = packed struct(u8) {
    pad: u3 = 0, // cgb only pallet
    bank: u1 = 0,
    dmg_pallet: u1 = 0,
    x_flip: u1 = 0,
    y_flip: u1 = 0,
    priority: u1 = 0,
};

const Object = packed struct(u32) {
    y_pos: u8 = 0,
    x_pos: u8 = 0,
    tile_idx: u8 = 0,
    flags: ObjectAttributes = .{},
};

const Mode0 = struct {};

const Mode1 = struct {};

const Mode2 = struct {
    dots_left: u8 = 80,
};

const Mode3 = struct {
    // 172 dots is the bare minimum. Extra dots past will cause this value to go negative.
    // This value will be added to mode0 dots_left to shorten its total length.
    dots_left: i16 = 172,
    fetcher_state: PixelFetcherState = .{},
};

const PpuState = union(enum) {
    mode0: Mode0,
    mode1: Mode1,
    mode2: Mode2,
    mode3: Mode3,
};

const PixelFetcherState = struct {
    // The pixel fetcher has 4 states but it goes through two states every m-cycle
    // and this emulator only guarentees m-cycle accuracy. For this reason,
    // we use an enum to represent the first and second cycles.
    mode: enum { first_tick, second_tick } = .first_tick,
    tile_address: u16 = 0, // Base address of tiles being fetched
    x_offset: u8 = 0,
};

pub const Ppu = struct {
    state: PpuState = .{ .mode2 = .{} },
    lcdc: LcdControl = .{}, // control register
    stat: LcdStatus = .{}, // status register
    scy: u8 = 0, // vertical scroll register
    scx: u8 = 0, // horizontal scroll register
    ly: u8 = 0, // scanline register
    lyc: u8 = 0, // scanline control register
    dma: u8 = 0,
    bgp: u8 = 0,
    obp0: u8 = 0,
    obp1: u8 = 0,
    wx: u8 = 0,
    wy: u8 = 0,

    fetcher_state: PixelFetcherState = .{}, // State machine for pixel fetcher

    vram: [hw.Map.vram.size]u8 = [_]u8{0} ** hw.Map.vram.size,
    oam: [hw.Map.oam.size]u8 = [_]u8{0} ** hw.Map.oam.size,
    oam_buf: std.BoundedArray(Object, 10) = .{},
    bg_fifo: LinearFifo(u8, .{ .Static = 8 }) = LinearFifo(u8, .{ .Static = 8 }).init(),
    obj_fifo: LinearFifo(u8, .{ .Static = 8 }) = LinearFifo(u8, .{ .Static = 8 }).init(),

    interrupts: *Interrupts,

    pub fn init(self: *Ppu, interrupts: *Interrupts) void {
        self.* = Ppu{
            .interrupts = interrupts,
            .bg_fifo = LinearFifo(u8, .{ .Static = 8 }).init(),
            .obj_fifo = LinearFifo(u8, .{ .Static = 8 }).init(),
        };

        self.oam_buf = std.BoundedArray(Object, 10).init(0) catch unreachable;
    }

    pub fn tick(self: *Ppu) void {
        switch (self.state) {
            .mode2 => |*mode2| self.oamScanTick(&mode2.dots_left),
            .mode3 => |*mode3| self.pixelTransferTick(mode3),
            else => {},
        }
    }

    pub fn writeDma(self: *Ppu, value: u8) void {
        self.state = .{ .mode2 = .{} };
        self.dma = value;
    }

    fn getTileBaseAddress(self: *Ppu, tile_id: u8) u16 {
        return if (self.lcdc.bg_win_tiles == 1)
            // Unsigned mode ($8000 - $8FFF)
            0x8000 + (@as(u16, tile_id) * 16)
        else blk: {
            // Signed mode ($8800 - $97FF)
            const signed_id: i8 = @bitCast(tile_id);
            const result_i32: i32 = 0x9000 + (@as(i32, signed_id) * 16);
            break :blk @intCast(result_i32);
        };
    }

    fn fetchTile(self: *Ppu, fetcher_state: *PixelFetcherState) void {
        // Calculate tile id if its the first fetcher m-cycle
        switch (fetcher_state.mode) {
            .first_tick => {
                // Calculate tile ID
                const map_y = (self.scy + self.ly) & 0xFF;
                const map_x = (self.scx + fetcher_state.x_offset) & 0xFF;
                const grid_y = map_y >> 3; // divide by 8
                const grid_x = map_x >> 3;
                const map_base_address: u16 = if (self.lcdc.obj_size == 1) 0x9800 else 0x9c00;
                const tile_map_address = map_base_address + (grid_y * 32) + grid_x;
                const tile_id = self.vram[tile_map_address];

                // Calculate tile address
                const tile_base_address = self.getTileBaseAddress(tile_id);
                const row_offset = @as(u16, (map_y % 8) * 2);
                fetcher_state.tile_address = tile_base_address + row_offset - hw.Map.vram.start;

                // Don't bother reading the address since nothing gets pushed to the fifos
                // until next tick.
                fetcher_state.mode = .second_tick;
            },
            .second_tick => {
                const pixels = self.vram[fetcher_state.tile_address .. fetcher_state.tile_address + 1];
                self.bg_fifo.write(pixels) catch {
                    return;
                };
                self.fetcher_state.mode = .first_tick;
            },
        }
    }

    fn tickMultiplexer(self: *Ppu) void {
        _ = self;
    }

    fn oamScanTick(self: *Ppu, dots_left: *u8) void {
        if (self.oam_buf.len == 10) return;
        const index = (80 - dots_left.*) * 2;
        var objs = [2]Object{ .{}, .{} };
        // This will never go out of bounds because this state only lasts for 80 t-cycles 100% of the time.
        std.debug.assert(index + 4 < self.oam.len);
        objs[0] = @bitCast(self.oam[index..][0..4].*);
        objs[1] = @bitCast(self.oam[index + 4 ..][0..4].*);
        dots_left.* -= 4;

        const sprite_height: u8 = if (self.lcdc.obj_size == 0) 8 else 16;

        for (objs) |obj| {
            if (self.ly >= obj.y_pos - 16 and self.ly < obj.y_pos - 16 + sprite_height) {
                self.oam_buf.append(obj) catch {};
            }
        }

        if (dots_left.* == 0) {
            self.state = .{ .mode3 = .{} };
        }
    }

    fn pixelTransferTick(self: *Ppu, mode3: *Mode3) void {
        mode3.dots_left -= 4;
        self.fetchTile(&mode3.fetcher_state);
    }
};
