const std = @import("std");
const readInt = std.mem.readInt;
const writeInt = std.mem.writeInt;
const Mbc = @import("mbc").Mbc;
const Io = @import("io").Io;
const hw = @import("hw");
const interrupts = @import("interrupts");
const Interrupts = interrupts.Interrupts;
const InterruptBit = interrupts.InterruptBit;

pub const Mmu = struct {
    vram: [hw.Map.vram.size]u8 = [_]u8{0} ** hw.Map.vram.size,
    wram0: [hw.Map.wram0.size]u8 = [_]u8{0} ** hw.Map.wram0.size,
    wram1: [hw.Map.wramx.size]u8 = [_]u8{0} ** hw.Map.wramx.size,
    hram: [hw.Map.hram.size]u8 = [_]u8{0} ** hw.Map.hram.size,
    oam: [hw.Map.oam.size]u8 = [_]u8{0} ** hw.Map.oam.size,
    interrupts: *Interrupts,
    io: *Io,
    mbc: *Mbc,

    pub fn read(self: *const Mmu, address: u16, comptime T: type) T {
        return switch (T) {
            u8 => self.readU8(address),
            u16 => self.readU16(address),
            else => @compileError("Mmu.read only supports u8 and u16 types"),
        };
    }

    pub fn write(self: *Mmu, address: u16, value: anytype) void {
        const T = @TypeOf(value);
        if (T == u8 or T == comptime_int) {
            const val: u8 = @truncate(value);
            switch (address) {
                hw.Map.rom0.start...hw.Map.romx.end => self.mbc.write(address, val),
                hw.Map.vram.start...hw.Map.vram.end => self.vram[address - hw.Map.vram.start] = val,
                hw.Map.ext_ram.start...hw.Map.ext_ram.end => self.mbc.write(address, val),
                hw.Map.wram0.start...hw.Map.wram0.end => self.wram0[address - hw.Map.wram0.start] = val,
                hw.Map.wramx.start...hw.Map.wramx.end => self.wram1[address - hw.Map.wramx.start] = val,
                hw.Map.echo.start...hw.Map.echo.end => self.write(address - 0x2000, val),
                hw.Map.oam.start...hw.Map.oam.end => self.oam[address - hw.Map.oam.start] = val,
                hw.Map.io.start...hw.Map.io.end => self.io.write(address - hw.Map.io.start, val),
                hw.Map.hram.start...hw.Map.hram.end => self.hram[address - hw.Map.hram.start] = val,
                hw.Map.ie_reg => self.interrupts.ie = val,
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

    pub fn isAnyButtonPressed(self: *const Mmu) bool {
        const joypad = self.io.read(hw.Io.joyp - hw.Map.io.start);
        return (joypad & 0x0F) != 0x0F;
    }

    pub fn enableInterrupt(self: *Mmu, comptime interrupt: InterruptBit) void {
        self.interrupts.ie |= (@as(u8, 1) << @intFromEnum(interrupt));
    }

    pub fn clearInterrupt(self: *Mmu, comptime interrupt: InterruptBit) void {
        self.interrupts.ie &= ~(@as(u8, 1) << @intFromEnum(interrupt));
    }

    pub fn requestInterrupt(self: *Mmu, comptime interrupt: InterruptBit) void {
        self.interrupts.request(interrupt);
    }

    pub fn acknowledgeInterrupt(self: *Mmu, interrupt: InterruptBit) void {
        self.interrupts.acknowledge(interrupt);
    }

    pub fn checkInterrupt(self: *const Mmu, comptime interrupt: InterruptBit) bool {
        return (self.interrupts.ie >> @intFromEnum(interrupt) & 1) == 1;
    }

    pub fn getPendingInterrupt(self: *const Mmu) ?InterruptBit {
        return self.interrupts.getPending();
    }

    fn readU8(self: *const Mmu, address: u16) u8 {
        return switch (address) {
            hw.Map.rom0.start...hw.Map.romx.end => self.mbc.read(address),
            hw.Map.vram.start...hw.Map.vram.end => self.vram[address - hw.Map.vram.start],
            hw.Map.ext_ram.start...hw.Map.ext_ram.end => self.mbc.read(address),
            hw.Map.wram0.start...hw.Map.wram0.end => self.wram0[address - hw.Map.wram0.start],
            hw.Map.wramx.start...hw.Map.wramx.end => self.wram1[address - hw.Map.wramx.start],
            hw.Map.echo.start...hw.Map.echo.end => self.readU8(address - 0x2000),
            hw.Map.oam.start...hw.Map.oam.end => self.oam[address - hw.Map.oam.start],
            hw.Map.unusable.start...hw.Map.unusable.end => 0xFF,
            hw.Map.io.start...hw.Map.io.end => self.io.read(address - hw.Map.io.start),
            hw.Map.hram.start...hw.Map.hram.end => self.hram[address - hw.Map.hram.start],
            hw.Map.ie_reg => self.interrupts.ie,
        };
    }

    fn readU16(self: *const Mmu, address: u16) u16 {
        const low = self.readU8(address);
        const high = self.readU8(address + 1);
        return (@as(u16, high) << 8) | low;
    }
};
