const std = @import("std");
const readInt = std.mem.readInt;
const writeInt = std.mem.writeInt;

pub const InteruptBit = enum(u3) {
    vblank = 0,
    lcd = 1,
    timer = 2,
    serial = 3,
    joypad = 4,
};

// Memory map constants
const VRAM_START: u16 = 0x8000;
const VRAM_END: u16 = 0x9FFF;
const EXTRAM_START: u16 = 0xA000;
const EXTRAM_END: u16 = 0xBFFF;
const OAM_START: u16 = 0xFE00;
const OAM_END: u16 = 0xFE9F;
const DEVICE_START: u16 = 0xFF00;
const DEVICE_END: u16 = 0xFF7F;
const IE_ADDRESS: u16 = 0xFFFF;
const IF_ADDRESS: u16 = 0xFF0F;

pub const Memory = struct {
    ram: [0x10000]u8 = undefined,

    pub fn read(self: *Memory, address: u16, return_type: type) return_type {
        if (return_type == u8) {
            return self.ram[address];
        } else if (return_type == u16){
            return readInt(u16, self.ram[address..][0..2], .little);
        } else {
            @compileError("Memory.read only supports u8 and u16 types");
        }
    }

    pub fn write(self: *Memory, address: u16, value: anytype) void {
        const T = @TypeOf(value);
        if (T == u8) {
            self.ram[address] = value;
        } else if (T == u16) {
            writeInt(u16, self.ram[address..][0..2], value, .little);
        } else {
            @compileError("Memory.write only supports u8 and u16 types");
        }
    }

    pub fn loadRom(self: *Memory, rom: []u8) !void {
        // TODO: research how this actually needs to work
        self.ram = rom;
    }

    pub fn enableInturrupt(self: *Memory, comptime interrupt: InteruptBit) void {
        self.ram[IE_ADDRESS] |= (1 << @intFromEnum(interrupt));
    }

    pub fn clearInterrupt(self: *Memory, comptime interrupt: InteruptBit) void {
        self.ram[IE_ADDRESS] &= (0 << @intFromEnum(interrupt));
    }

    pub fn requestInterrupt(self: *Memory, comptime interrupt: InteruptBit) void {
        self.ram[IF_ADDRESS] |= (1 << @intFromEnum(interrupt));
    }

    pub fn acknowledgeInterrupt(self: *Memory, comptime interrupt: InteruptBit) void {
        self.ram[IF_ADDRESS] &= (0 << @intFromEnum(interrupt));
    }

    pub fn checkInterrupt(self: *Memory, comptime interrupt: InteruptBit) bool {
        if ((self.ram[IE_ADDRESS] >> @intFromEnum(interrupt) & 1) == 1) {
            return true;
        }
        return false;
    }

    pub fn getPendingInterrupt(self: *Memory) ?InteruptBit {
        const enabled = self.ram[IE_ADDRESS];
        const requested = self.ram[IF_ADDRESS];
        const pending = enabled & requested;

        // Flags have a priority order.
        if (pending & 0b00001 != 0) return .vblank;
        if (pending & 0b00010 != 0) return .lcd;
        if (pending & 0b00100 != 0) return .timer;
        if (pending & 0b01000 != 0) return .serial;
        if (pending & 0b10000 != 0) return .joypad;

        return null;
    }
};
