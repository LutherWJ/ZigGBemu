const ppu = @import("ppu");
const Object = ppu.Object;
const SpritePixel = ppu.SpritePixel;

const PixelFifoError = error{
    BufferFull,
    BufferEmpty,
};

pub const BackgroundFifo = struct {
    buffer: [16]u2 = undefined,
    head: u4 = 0,
    tail: u4 = 0,
    count: u4 = 0,

    pub fn pushRow(self: *BackgroundFifo, row: u16) PixelFifoError!void {
        inline for (0..8) |i| {
            const shift = i * 2;
            const pixel: u2 = @truncate(row >> shift);

            if (self.count + 8 > self.buffer.len) return PixelFifoError.BufferFull;
            self.buffer[self.tail] = pixel;
            self.tail +%= 1;
        }
        self.count += 8;
    }

    pub fn popPixel(self: *BackgroundFifo) PixelFifoError!u2 {
        if (self.count == 0) return PixelFifoError.BufferEmpty;
        const pixel = self.buffer[self.head];
        self.head +%= 1;
        self.count -= 1;
        return pixel;
    }

    pub fn clear(self: *BackgroundFifo) void {
        self.head = 0;
        self.tail = 0;
        self.count = 0;
    }
};

pub const SpriteFifo = struct {
    buffer: [8]SpritePixel = undefined,
    head: u3 = 0,
    tail: u3 = 0,
    count: u4 = 0,

    pub fn pushRow(self: *SpriteFifo, row: [8]SpritePixel) void {
        for (row, 0..) |new_pixel, i| {
            const ring_idx = self.head +% @as(u3, @intCast(i));
            const old_pixel = self.buffer[ring_idx];

            if (new_pixel.pixel == 0) continue;

            if (i >= self.count) {
                self.buffer[ring_idx] = new_pixel;
                continue;
            }

            if (new_pixel.x_pos < old_pixel.x_pos) {
                self.buffer[ring_idx] = new_pixel;
            } else if (new_pixel.x_pos == old_pixel.x_pos) {
                if (new_pixel.oam_idx < old_pixel.oam_idx) {
                    self.buffer[ring_idx] = new_pixel;
                }
            }
        }
        self.count = 8;
    }

    pub fn popPixel(self: *SpriteFifo) ?SpritePixel {
        if (self.count == 0) return null;
        const pixel = self.buffer[self.head];
        self.head +%= 1;
        self.count -= 1;
        return pixel;
    }

    pub fn clear(self: *SpriteFifo) void {
        self.head = 0;
        self.tail = 0;
        self.count = 0;
    }
};
