const std = @import("std");
const Allocator = std.mem.Allocator;
const MBC0 = @import("mbc0.zig").MBC0;
const MBC1 = @import("mbc1.zig").MBC1;
const Header = @import("header.zig").Header;

pub const MBC = union(enum) {
    mbc0: MBC0,
    mbc1: MBC1,

    pub fn init(allocator: Allocator, rom: []const u8) !MBC {
        const header = Header.parse(rom);
        const ram_size = header.getRamSize();
        const ram = if (ram_size > 0) try allocator.alloc(u8, ram_size) else &[_]u8{};
        @memset(ram, 0);

        return switch (header.cart_type) {
            0x00 => .{ .mbc0 = .{ .rom = rom, .ram = ram } },
            0x01...0x03 => .{ .mbc1 = .{ .rom = rom, .ram = ram } },
            else => {
                std.log.err("Unsupported Cartridge Type: 0x{X:0>2}", .{header.cart_type});
                return error.UnsupportedMbcType;
            },
        };
    }

    pub fn read(self: MBC, address: u16) u8 {
        return switch (self) {
            .mbc0 => |m| m.read(address),
            .mbc1 => |m| m.read(address),
        };
    }

    pub fn write(self: *MBC, address: u16, value: u8) void {
        switch (self.*) {
            .mbc0 => |*m| m.write(address, value),
            .mbc1 => |*m| m.write(address, value),
        }
    }
};
