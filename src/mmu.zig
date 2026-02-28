const std = @import("std");
const readInt = std.mem.readInt;
const writeInt = std.mem.writeInt;

pub const InterruptBit = enum(u3) {
    vblank = 0,
    lcd = 1,
    timer = 2,
    serial = 3,
    joypad = 4,
};

// Memory map constants
const ROM0_START: u16 = 0x0000;
const ROM0_END: u16 = 0x3FFF;
const ROM0_SIZE = ROM0_END - ROM0_START + 1;

const ROMX_START: u16 = 0x4000;
const ROMX_END: u16 = 0x7FFF;
const ROMX_SIZE = ROMX_END - ROMX_START + 1;

const VRAM_START: u16 = 0x8000;
const VRAM_END: u16 = 0x9FFF;
const VRAM_SIZE = VRAM_END - VRAM_START + 1;

const EXTRAM_START: u16 = 0xA000;
const EXTRAM_END: u16 = 0xBFFF;
const EXTRAM_SIZE = EXTRAM_END - EXTRAM_START + 1;

const WRAM0_START: u16 = 0xC000;
const WRAM0_END: u16 = 0xCFFF;
const WRAM0_SIZE = WRAM0_END - WRAM0_START + 1;

const WRAMX_START: u16 = 0xD000;
const WRAMX_END: u16 = 0xDFFF;
const WRAMX_SIZE = WRAMX_END - WRAMX_START + 1;

const ECHO_START: u16 = 0xE000;
const ECHO_END: u16 = 0xFDFF;
const ECHO_SIZE = ECHO_END - ECHO_START + 1;

const OAM_START: u16 = 0xFE00;
const OAM_END: u16 = 0xFE9F;
const OAM_SIZE = OAM_END - OAM_START + 1;

const UNUSABLE_START: u16 = 0xFEA0;
const UNUSABLE_END: u16 = 0xFEFF;

const IO_START: u16 = 0xFF00;
const IO_END: u16 = 0xFF7F;
const IO_SIZE = IO_END - IO_START + 1;

const HRAM_START: u16 = 0xFF80;
const HRAM_END: u16 = 0xFFFE;
const HRAM_SIZE = HRAM_END - HRAM_START + 1;

const IE_ADDRESS: u16 = 0xFFFF;
const IE_REG = IE_ADDRESS;
const IF_ADDRESS: u16 = 0xFF0F;

pub const MMU = struct {
    ie_reg: u8 = 0,
    if_reg: u8 = 0,
    vram: [VRAM_SIZE]u8 = [_]u8{0} ** VRAM_SIZE,
    wram0: [WRAM0_SIZE]u8 = [_]u8{0} ** WRAM0_SIZE,
    wram1: [WRAMX_SIZE]u8 = [_]u8{0} ** WRAMX_SIZE,
    hram: [HRAM_SIZE]u8 = [_]u8{0} ** HRAM_SIZE,
    oam: [OAM_SIZE]u8 = [_]u8{0} ** OAM_SIZE,

    pub fn read(self: *MMU, address: u16, comptime T: type) T {
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
                VRAM_START...VRAM_END => self.vram[address - VRAM_START] = val,
                WRAM0_START...WRAM0_END => self.wram0[address - WRAM0_START] = val,
                WRAMX_START...WRAMX_END => self.wram1[address - WRAMX_START] = val,
                ECHO_START...ECHO_END => self.write(address - 0x2000, val),
                OAM_START...OAM_END => self.oam[address - OAM_START] = val,
                HRAM_START...HRAM_END => self.hram[address - HRAM_START] = val,
                IO_START...IO_END => switch (address) {
                    IF_ADDRESS => self.if_reg = val,
                    else => {}, // TODO: handle other I/O
                },
                IE_REG => self.ie_reg = val,
                else => {}, // TODO: handle other regions
            }
        } else if (T == u16) {
            self.write(address, @as(u8, @truncate(value)));
            self.write(address + 1, @as(u8, @truncate(value >> 8)));
        } else {
            @compileError("Memory.write only supports u8 and u16 types");
        }
    }

    pub fn loadRom(self: *MMU, rom: []u8) !void {
        _ = self;
        _ = rom;
        // TODO: implement cartridge loading
    }

    pub fn isAnyButtonPressed(self: *MMU) bool {
        _ = self;
        // TODO: implement joypad register read
        return false;
    }

    pub fn enableInturrupt(self: *MMU, comptime interrupt: InterruptBit) void {
        self.ie_reg |= (1 << @intFromEnum(interrupt));
    }

    pub fn clearInterrupt(self: *MMU, comptime interrupt: InterruptBit) void {
        self.ie_reg &= ~(@as(u8, 1) << @intFromEnum(interrupt));
    }

    pub fn requestInterrupt(self: *MMU, comptime interrupt: InterruptBit) void {
        self.if_reg |= (@as(u8, 1) << @intFromEnum(interrupt));
    }

    pub fn acknowledgeInterrupt(self: *MMU, interrupt: InterruptBit) void {
        self.if_reg &= ~(@as(u8, 1) << @intFromEnum(interrupt));
    }

    pub fn checkInterrupt(self: *MMU, comptime interrupt: InterruptBit) bool {
        if ((self.ie_reg >> @intFromEnum(interrupt) & 1) == 1) {
            return true;
        }
        return false;
    }

    pub fn getPendingInterrupt(self: *MMU) ?InterruptBit {
        const enabled = self.ie_reg;
        const requested = self.if_reg;
        const pending = enabled & requested;

        // Flags have a priority order.
        if (pending & 0b00001 != 0) return .vblank;
        if (pending & 0b00010 != 0) return .lcd;
        if (pending & 0b00100 != 0) return .timer;
        if (pending & 0b01000 != 0) return .serial;
        if (pending & 0b10000 != 0) return .joypad;

        return null;
    }

    fn readU8(self: *MMU, address: u16) u8 {
        return switch (address) {
            ROM0_START...ROM0_END => 0, // TODO: handle ROM
            ROMX_START...ROMX_END => 0,
            VRAM_START...VRAM_END => self.vram[address - VRAM_START],
            EXTRAM_START...EXTRAM_END => 0,
            WRAM0_START...WRAM0_END => self.wram0[address - WRAM0_START],
            WRAMX_START...WRAMX_END => self.wram1[address - WRAMX_START],
            ECHO_START...ECHO_END => self.readU8(address - 0x2000),
            OAM_START...OAM_END => self.oam[address - OAM_START],
            UNUSABLE_START...UNUSABLE_END => 0xFF,
            IO_START...IO_END => switch (address) {
                IF_ADDRESS => self.if_reg,
                else => 0,
            },
            HRAM_START...HRAM_END => self.hram[address - HRAM_START],
            IE_REG => self.ie_reg,
        };
    }

    fn readU16(self: *MMU, address: u16) u16 {
        const low = self.readU8(address);
        const high = self.readU8(address + 1);
        return (@as(u16, high) << 8) | low;
    }
};
