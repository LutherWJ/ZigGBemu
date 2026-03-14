const std = @import("std");
const readInt = std.mem.readInt;
const writeInt = std.mem.writeInt;
const MBC = @import("mbc.zig").MBC;
const hw = @import("constants.zig");

pub const InterruptBit = enum(u3) {
    vblank = 0,
    lcd = 1,
    timer = 2,
    serial = 3,
    joypad = 4,
};

pub const Joypad = struct {
    a: bool = false,
    b: bool = false,
    select: bool = false,
    start: bool = false,
    right: bool = false,
    left: bool = false,
    up: bool = false,
    down: bool = false,
};

pub const MMU = struct {
    vram: [hw.map.vram.size]u8 = [_]u8{0} ** hw.map.vram.size,
    wram0: [hw.map.wram0.size]u8 = [_]u8{0} ** hw.map.wram0.size,
    wram1: [hw.map.wramx.size]u8 = [_]u8{0} ** hw.map.wramx.size,
    hram: [hw.map.hram.size]u8 = [_]u8{0} ** hw.map.hram.size,
    oam: [hw.map.oam.size]u8 = [_]u8{0} ** hw.map.oam.size,
    io: [hw.map.io.size]u8 = [_]u8{0} ** hw.map.io.size,
    ie_reg: u8 = 0,
    joypad: Joypad = .{},
    mbc: MBC,

    pub fn read(self: *const MMU, address: u16, comptime T: type) T {
        return switch (T) {
            u8 => self.readU8(address),
            u16 => self.readU16(address),
            else => @compileError("MMU.read only supports u8 and u16 types"),
        };
    }

    pub fn write(self: *MMU, address: u16, value: anytype) void {
        const T = @TypeOf(value);
        if (T == u8 or T == comptime_int) {
            const val: u8 = @truncate(value);
            switch (address) {
                hw.map.rom0.start...hw.map.romx.end => self.mbc.write(address, val),
                hw.map.vram.start...hw.map.vram.end => self.vram[address - hw.map.vram.start] = val,
                hw.map.ext_ram.start...hw.map.ext_ram.end => self.mbc.write(address, val),
                hw.map.wram0.start...hw.map.wram0.end => self.wram0[address - hw.map.wram0.start] = val,
                hw.map.wramx.start...hw.map.wramx.end => self.wram1[address - hw.map.wramx.start] = val,
                hw.map.echo.start...hw.map.echo.end => self.write(address - 0x2000, val),
                hw.map.oam.start...hw.map.oam.end => self.oam[address - hw.map.oam.start] = val,
                hw.map.io.start...hw.map.io.end => switch (address) {
                    hw.io.joyp => self.io[0] = (val & 0x30) | 0b11000000,
                    else => self.io[address - hw.map.io.start] = val,
                },
                hw.map.hram.start...hw.map.hram.end => self.hram[address - hw.map.hram.start] = val,
                hw.map.ie_reg => self.ie_reg = val,
                else => {
                    std.log.warn("Attempted to write to unimplemented memory region at address {x}", .{address});
                },
            }
        } else if (T == u16) {
            self.write(address, @as(u8, @truncate(value)));
            self.write(address + 1, @as(u8, @truncate(value >> 8)));
        } else {
            @compileError("Memory.write only supports u8, 16 and comptime_int");
        }
    }

    pub fn isAnyButtonPressed(self: *const MMU) bool {
        const joypad = self.io[hw.io.joyp - hw.map.io.start];
        return (joypad & 0x0F) != 0x0F;
    }

    pub fn loadRom(self: *MMU, rom: []u8) !void {
        _ = self;
        _ = rom;
        // TODO: implement cartridge loading
    }

    pub fn enableInturrupt(self: *MMU, comptime interrupt: InterruptBit) void {
        self.ie_reg |= (1 << @intFromEnum(interrupt));
    }

    pub fn clearInterrupt(self: *MMU, comptime interrupt: InterruptBit) void {
        self.ie_reg &= ~(@as(u8, 1) << @intFromEnum(interrupt));
    }

    pub fn requestInterrupt(self: *MMU, comptime interrupt: InterruptBit) void {
        self.io[hw.io.if_reg - hw.map.io.start] |= (@as(u8, 1) << @intFromEnum(interrupt));
    }

    pub fn acknowledgeInterrupt(self: *MMU, interrupt: InterruptBit) void {
        self.io[hw.io.if_reg - hw.map.io.start] &= ~(@as(u8, 1) << @intFromEnum(interrupt));
    }

    pub fn checkInterrupt(self: *const MMU, comptime interrupt: InterruptBit) bool {
        if ((self.ie_reg >> @intFromEnum(interrupt) & 1) == 1) {
            return true;
        }
        return false;
    }

    pub fn getPendingInterrupt(self: *const MMU) ?InterruptBit {
        const enabled = self.ie_reg;
        const requested = self.io[hw.io.if_reg - hw.map.io.start];
        const pending = enabled & requested;

        // Flags have a priority order.
        if (pending & 0b00001 != 0) return .vblank;
        if (pending & 0b00010 != 0) return .lcd;
        if (pending & 0b00100 != 0) return .timer;
        if (pending & 0b01000 != 0) return .serial;
        if (pending & 0b10000 != 0) return .joypad;

        return null;
    }

    fn readU8(self: *const MMU, address: u16) u8 {
        return switch (address) {
            hw.map.rom0.start...hw.map.romx.end => self.mbc.read(address),
            hw.map.vram.start...hw.map.vram.end => self.vram[address - hw.map.vram.start],
            hw.map.ext_ram.start...hw.map.ext_ram.end => self.mbc.read(address),
            hw.map.wram0.start...hw.map.wram0.end => self.wram0[address - hw.map.wram0.start],
            hw.map.wramx.start...hw.map.wramx.end => self.wram1[address - hw.map.wramx.start],
            hw.map.echo.start...hw.map.echo.end => self.readU8(address - 0x2000),
            hw.map.oam.start...hw.map.oam.end => self.oam[address - hw.map.oam.start],
            hw.map.unusable.start...hw.map.unusable.end => 0xFF,
            hw.map.io.start...hw.map.io.end => switch (address) {
                hw.io.joyp => {
                    var val: u8 = self.io[0] | 0xCF; // bits 6-7 are 1, bits 0-3 are 1 by default
                    if ((self.io[0] & 0x20) == 0) {
                        if (self.joypad.start) val &= ~@as(u8, 0x08);
                        if (self.joypad.select) val &= ~@as(u8, 0x04);
                        if (self.joypad.b) val &= ~@as(u8, 0x02);
                        if (self.joypad.a) val &= ~@as(u8, 0x01);
                    }
                    if ((self.io[0] & 0x10) == 0) {
                        if (self.joypad.down) val &= ~@as(u8, 0x08);
                        if (self.joypad.up) val &= ~@as(u8, 0x04);
                        if (self.joypad.left) val &= ~@as(u8, 0x02);
                        if (self.joypad.right) val &= ~@as(u8, 0x01);
                    }
                    return val;
                },
                else => self.io[address - hw.map.io.start],
            },
            hw.map.hram.start...hw.map.hram.end => self.hram[address - hw.map.hram.start],
            hw.map.ie_reg => self.ie_reg,
        };
    }

    fn readU16(self: *const MMU, address: u16) u16 {
        const low = self.readU8(address);
        const high = self.readU8(address + 1);
        return (@as(u16, high) << 8) | low;
    }
};
