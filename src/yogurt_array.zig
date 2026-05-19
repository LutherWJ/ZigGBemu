// Turns out bounded arrays have a lot of overhead associated with them, simple array wrapper to improve performance.
const std = @import("std");

const YogurtErrors = error{
    YogurtFull,
};

// That shits the yogurt array.
pub fn YogurtArray(comptime T: type, comptime length: usize) type {
    return struct {
        len: usize = 0,
        buffer: [length]T = [_]T{undefined} ** length,
        size: usize = length,

        const Self = @This();

        pub fn append(self: *Self, value: T) !void {
            if (self.size == self.len) return YogurtErrors.YogurtFull;
            self.buffer[self.len] = value;
            self.len += 1;
        }

        pub fn clear(self: *Self) void {
            self.len = 0;
        }

        pub fn get(self: *Self, index: usize) *const T {
            std.debug.assert(index < self.len);
            return &self.buffer[index];
        }
    };
}
