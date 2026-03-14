const std = @import("std");
const Allocator = std.mem.Allocator;
const Mbc0 = @import("mbc0.zig").Mbc0;
const Mbc1 = @import("mbc1.zig").Mbc1;
const Header = @import("header.zig").Header;

pub const Mbc = union(enum) {
    mbc0: Mbc0,
    mbc1: Mbc1,

    pub fn init(allocator: std.mem.Allocator, rom: []const u8) !Mbc {
        const header = Header.parse(rom);

        return switch (header.cart_type) {
            0x00 => .{ .mbc0 = .{ .rom = rom } },
            0x01...0x03 => blk: {
                var h = header;
                const ram_size = h.getRamSize();
                const ram = try allocator.alloc(u8, ram_size);
                break :blk .{ .mbc1 = .{ .rom = rom, .ram = ram } };
            },
            else => {
                std.log.err("Unsupported Cartridge Type: 0x{X:0>2}", .{header.cart_type});
                return error.UnsupportedMbcType;
            },
        };
    }

    pub fn read(self: *const Mbc, address: u16) u8 {
        return switch (self.*) {
            .mbc0 => |*m| m.read(address),
            .mbc1 => |*m| m.read(address),
        };
    }

    pub fn write(self: *Mbc, address: u16, value: u8) void {
        switch (self.*) {
            .mbc0 => |*m| m.write(address, value),
            .mbc1 => |*m| m.write(address, value),
        }
    }
};
