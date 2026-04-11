const hw = @import("hw");

pub const Display = struct {
    frame_buf: [hw.Lcd.area]u32 = [_]u32{0} ** hw.Lcd.area,

    pub fn drawPixel(self: *Display, pixel: u2, pallete: u8, x: u8, y: u8) void {
        _ = self;
        _ = pixel;
        _ = pallete;
        _ = x;
        _ = y;
    }
};
