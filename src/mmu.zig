const std = @import("std");
const Mbc = @import("mbc").Mbc;
const Io = @import("io").Io;
const hw = @import("hw");
const interrupts = @import("interrupts");
const Interrupts = interrupts.Interrupts;
const InterruptBit = interrupts.InterruptBit;
const Timer = @import("timer").Timer;
const Ppu = @import("ppu").Ppu;

pub const Mmu = struct {
    wram0: [hw.Map.wram0.size]u8 = [_]u8{0} ** hw.Map.wram0.size,
    wram1: [hw.Map.wramx.size]u8 = [_]u8{0} ** hw.Map.wramx.size,
    hram: [hw.Map.hram.size]u8 = [_]u8{0} ** hw.Map.hram.size,
    interrupts: *Interrupts,
    timer: *Timer,
    io: *Io,
    mbc: *Mbc,
    ppu: *Ppu,

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
            self.timer.tick();
            switch (address) {
                hw.Map.rom0.start...hw.Map.romx.end => self.mbc.write(address, val),
                hw.Map.vram.start...hw.Map.vram.end => self.ppu.writeVram(address, val),
                hw.Map.ext_ram.start...hw.Map.ext_ram.end => self.mbc.write(address, val),
                hw.Map.wram0.start...hw.Map.wram0.end => self.wram0[address - hw.Map.wram0.start] = val,
                hw.Map.wramx.start...hw.Map.wramx.end => self.wram1[address - hw.Map.wramx.start] = val,
                hw.Map.echo.start...hw.Map.echo.end => self.write(address - 0x2000, val),
                hw.Map.oam.start...hw.Map.oam.end => self.ppu.writeOam(address, val),
                hw.Map.io.start...hw.Map.io.end => self.io.write(address, val),
                hw.Map.hram.start...hw.Map.hram.end => self.hram[address - hw.Map.hram.start] = val,
                hw.Map.ie_reg => self.interrupts.ie = val,
                else => {
                    //std.log.warn("Attempted to write to unimplemented memory region at address {x}", .{address});
                },
            }
        } else if (T == u16) {
            self.write(address, @as(u8, @truncate(value)));
            self.write(address + 1, @as(u8, @truncate(value >> 8)));
        } else {
            @compileError("Memory.write only supports u8, 16 and comptime_int");
        }
    }

    /// Used to initialize memory at boot time without ticking the timer
    /// Only supports memory regions written to at boot time.
    pub fn bootWrite(self: *Mmu, address: u16, value: u8) void {
        switch (address) {
            hw.Map.vram.start...hw.Map.vram.end => self.ppu.vram[address - hw.Map.vram.start] = value,
            hw.Map.wram0.start...hw.Map.wram0.end => self.wram0[address - hw.Map.wram0.start] = value,
            hw.Map.wramx.start...hw.Map.wramx.end => self.wram1[address - hw.Map.wramx.start] = value,
            hw.Map.oam.start...hw.Map.oam.end => self.ppu.oam[address - hw.Map.oam.start] = value,
            hw.Map.io.start...hw.Map.io.end => self.io.write(address, value),
            hw.Map.hram.start...hw.Map.hram.end => self.hram[address - hw.Map.hram.start] = value,
            hw.Map.ie_reg => self.interrupts.ie = value,
            else => unreachable,
        }
    }

    pub fn isAnyButtonPressed(self: *const Mmu) bool {
        const joypad = self.io.read(hw.Io.joyp);
        return (joypad & 0x0F) != 0x0F;
    }

    fn readU8(self: *const Mmu, address: u16) u8 {
        self.timer.tick();
        return switch (address) {
            hw.Map.rom0.start...hw.Map.romx.end => self.mbc.read(address),
            hw.Map.vram.start...hw.Map.vram.end => self.ppu.readVram(address),
            hw.Map.ext_ram.start...hw.Map.ext_ram.end => self.mbc.read(address),
            hw.Map.wram0.start...hw.Map.wram0.end => self.wram0[address - hw.Map.wram0.start],
            hw.Map.wramx.start...hw.Map.wramx.end => self.wram1[address - hw.Map.wramx.start],
            hw.Map.echo.start...hw.Map.echo.end => self.readU8(address - 0x2000),
            hw.Map.oam.start...hw.Map.oam.end => self.ppu.readVram(address),
            hw.Map.unusable.start...hw.Map.unusable.end => 0xFF,
            hw.Map.io.start...hw.Map.io.end => self.io.read(address),
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
