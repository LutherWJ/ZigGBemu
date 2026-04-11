const std = @import("std");
const Interrupts = @import("interrupts").Interrupts;
const hw = @import("hw");
const PixelFifos = @import("pixel_fifo");
const Display = @import("display").Display;
const BackgroundFifo = PixelFifos.BackgroundFifo;
const SpriteFifo = PixelFifos.SpriteFifo;
const writeInt = std.mem.writeInt;
const readInt = std.mem.readInt;

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
    dmg_pallete: u1 = 0,
    x_flip: u1 = 0,
    y_flip: u1 = 0,
    priority: u1 = 0,
};

pub const Object = packed struct(u32) {
    y_pos: u8 = 0,
    x_pos: u8 = 0,
    tile_id: u8 = 0,
    flags: ObjectAttributes = .{},
};

pub const SpritePixel = struct {
    pixel: u2 = 0,
    x_pos: u8 = 0,
    oam_idx: u8 = 0,
    bg_priority: u1 = 0,
    pallete: u1 = 0,
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

const FetcherModes = enum {
    first_tick_bg,
    second_tick_bg,
    first_tick_sprt,
    second_tick_sprt,
};

const PixelFetcherState = struct {
    // The pixel fetcher has 4 states but it goes through two states every m-cycle
    // and this emulator only guarentees m-cycle accuracy. For this reason,
    // we use an enum to represent the first and second cycles.
    mode: FetcherModes = .first_tick_bg,
    suspended_mode: FetcherModes = .first_tick_bg,
    pix_address: u16 = 0, // Base address of pixels being fetched.
    focused_obj_idx: ?u8 = 0, // Multiplexer sets this value when it discovers a sprite needs to be rendered.
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
    bg_fifo: BackgroundFifo = .{},
    sprite_fifo: SpriteFifo = .{},
    framebuf: [hw.Lcd.area]u8 = [_]u8{0} ** (hw.Lcd.area),

    interrupts: *Interrupts,
    display: *Display,

    pub fn init(self: *Ppu, interrupts: *Interrupts, display: *Display) void {
        self.* = Ppu{
            .interrupts = interrupts,
            .display = display,
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
            .first_tick_bg => {
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
                fetcher_state.pix_address = tile_base_address + row_offset - hw.Map.vram.start;

                // Don't bother reading the address since nothing gets pushed to the fifos until next tick
                fetcher_state.mode = .second_tick_bg;
            },
            .second_tick_bg => {
                const byte1 = self.vram[fetcher_state.pix_address];
                const byte2 = self.vram[fetcher_state.pix_address + 1];
                const pixels = interleaveBytes(byte1, byte2);
                self.bg_fifo.pushRow(pixels) catch {
                    return;
                };
                self.fetcher_state.mode = .first_tick_bg;
            },
            .first_tick_sprt => {
                // Calculate which row we're drawing.
                const obj = self.oam_buf.get(fetcher_state.focused_obj_idx.?);
                const true_y: i32 = @as(i32, obj.y_pos) - 16;
                const current_line: i32 = @as(i32, self.ly);
                var sprite_row: u16 = @as(u16, @intCast(current_line - true_y));

                // Check if its flipped and recalculate
                const y_flip = obj.flags.y_flip == 1;
                const sprite_height: u8 = if (self.lcdc.obj_size == 1) 16 else 8;
                if (y_flip) sprite_row ^= (sprite_height - 1);

                // Find the pixel address
                const base_address = self.getTileBaseAddress(obj.tile_id);
                fetcher_state.pix_address = base_address + (sprite_row * 2) - hw.Map.vram.start;

                fetcher_state.mode = .second_tick_sprt;
            },
            .second_tick_sprt => {
                const byte1 = self.vram[fetcher_state.pix_address];
                const byte2 = self.vram[fetcher_state.pix_address + 1];
                const pixels = interleaveBytes(byte1, byte2);

                const obj = self.oam_buf.get(fetcher_state.focused_obj_idx.?);
                var sprite_pixels: [8]SpritePixel = undefined;
                for (0..8) |i| {
                    const offset: u4 = @truncate(i * 2);
                    const pixel: u2 = @truncate(pixels >> offset);
                    sprite_pixels[i] = .{
                        .pixel = pixel,
                        .x_pos = obj.x_pos,
                        .oam_idx = fetcher_state.focused_obj_idx.?,
                        .bg_priority = obj.flags.priority,
                        .pallete = obj.flags.dmg_pallete,
                    };
                }

                self.sprite_fifo.pushRow(sprite_pixels);
                std.debug.assert(fetcher_state.suspended_mode != .first_tick_sprt);
                std.debug.assert(fetcher_state.suspended_mode != .second_tick_sprt);
                fetcher_state.mode = fetcher_state.suspended_mode;
            },
        }
    }

    fn oamScanTick(self: *Ppu, dots_left: *u8) void {
        if (self.oam_buf.len >= 10) return;
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
                self.oam_buf.append(obj) catch unreachable;
            }
        }

        if (dots_left.* == 0) {
            self.state = .{ .mode3 = .{} };
        }
    }

    // Iterates through selected objects looking for one that starts on the current pixel.
    // If it finds one, it sets the fetcher state to sprite mode and tells it what index
    // to focus in the oam_buf. Multiple sprites can have the same coordinates so we need
    // to be extremely careful to not fetch the same sprite more than once.
    fn checkForSprite(self: *Ppu, fetcher_state: *PixelFetcherState) bool {
        const obj_idx = fetcher_state.focused_obj_idx;
        for (0..self.oam_buf.len) |i| {
            const obj = self.oam_buf.get(i);
            const target_x = fetcher_state.x_offset + 8;
            // make sure we don't fetch the same object infinitely
            if (obj.x_pos == target_x and (obj_idx == null or obj_idx.? < i)) {
                fetcher_state.focused_obj_idx = @intCast(i);
                fetcher_state.suspended_mode = fetcher_state.mode;
                fetcher_state.mode = .first_tick_sprt;
                return true;
            }
        }
        return false;
    }

    fn pixelTransferTick(self: *Ppu, mode3: *Mode3) void {
        mode3.dots_left -= 4;
        self.fetchTile(&mode3.fetcher_state);

        // Multiplexing!
        for (0..4) |_| {
            if (self.checkForSprite(&mode3.fetcher_state)) {
                return;
            }

            var prio_pixel: u2 = undefined;
            var prio_pallete: u8 = undefined;

            const bg_pixel = self.bg_fifo.popPixel() catch return;
            if (self.sprite_fifo.popPixel()) |sprite_pixel| {
                if ((sprite_pixel.bg_priority == 0 or bg_pixel == 0) and sprite_pixel.pixel != 0) {
                    prio_pixel = sprite_pixel.pixel;
                    prio_pallete = if (sprite_pixel.pallete == 0) self.obp0 else self.obp1;
                }
            }
            prio_pixel = bg_pixel;
            prio_pallete = self.bgp;

            self.display.drawPixel(prio_pixel, prio_pallete, mode3.fetcher_state.x_offset, self.ly);
            mode3.fetcher_state.x_offset += 1;
            // Set to null to make sure we don't get a false comparison next time we check
            // oam_buf for a new index.
            mode3.fetcher_state.focused_obj_idx = null;

            if (mode3.fetcher_state.x_offset == 160) {
                self.state = .{ .mode0 = .{} };
                self.bg_fifo.clear();
                self.sprite_fifo.clear();
                return;
            }
        }
    }
};

fn interleaveBytes(byte1: u8, byte2: u8) u16 {
    var x = @as(u16, byte1);
    x = (x | (x << 4)) & 0x0F0F;
    x = (x | (x << 2)) & 0x3333;
    x = (x | (x << 1)) & 0x5555;

    var y = @as(u16, byte2);
    y = (y | (y << 4)) & 0x0F0F;
    y = (y | (y << 2)) & 0x3333;
    y = (y | (y << 1)) & 0x5555;

    return x | (y << 1);
}
