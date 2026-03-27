const std = @import("std");
const Cpu = @import("cpu").Cpu;
const Mmu = @import("mmu").Mmu;
const Mbc = @import("mbc").Mbc;
const Timer = @import("timer").Timer;
const Joypad = @import("joypad").Joypad;
const Io = @import("io").Io;
const Interrupts = @import("interrupts").Interrupts;
const Sdt = @import("sdt").Sdt;
const Ppu = @import("ppu").Ppu;
const hw = @import("hw");

pub const Emulator = struct {
    _arena: std.heap.ArenaAllocator,
    _frame_sync: u8 = 0, // Holds how many cycles we overshot cyclesPerFrame by
    interrupts: *Interrupts,
    timer: *Timer,
    joypad: *Joypad,
    sdt: *Sdt,
    io: *Io,
    mmu: *Mmu,
    mbc: *Mbc,
    cpu: *Cpu,
    ppu: *Ppu,

    pub fn init(allocator: std.mem.Allocator, rom_buf: []const u8) !*Emulator {
        var arena = std.heap.ArenaAllocator.init(allocator);
        errdefer arena.deinit();
        const aa = arena.allocator();

        const rom = try aa.dupe(u8, rom_buf);

        const emu = try aa.create(Emulator);
        emu._arena = arena;

        emu.interrupts = try aa.create(Interrupts);
        emu.interrupts.* = .{};

        emu.sdt = try aa.create(Sdt);
        emu.sdt.* = .{ .interrupts = emu.interrupts };

        emu.timer = try aa.create(Timer);
        emu.timer.* = .{ .interrupts = emu.interrupts, .sdt = emu.sdt };

        emu.joypad = try aa.create(Joypad);
        emu.joypad.* = .{};

        emu.io = try aa.create(Io);
        emu.io.* = .{
            .timer = emu.timer,
            .joypad = emu.joypad,
            .interrupts = emu.interrupts,
            .sdt = emu.sdt,
        };

        emu.mbc = try aa.create(Mbc);
        emu.mbc.* = try Mbc.init(aa, rom);

        emu.mmu = try aa.create(Mmu);
        emu.mmu.* = .{
            .interrupts = emu.interrupts,
            .timer = emu.timer,
            .io = emu.io,
            .mbc = emu.mbc,
        };

        emu.cpu = try aa.create(Cpu);
        emu.cpu.* = .{
            .mmu = emu.mmu,
            .interrupts = emu.interrupts,
            .timer = emu.timer,
            .sdt = emu.sdt,
        };
        emu.cpu.boot();

        emu.ppu = try aa.create(Ppu);
        emu.ppu.init(emu.interrupts);

        return emu;
    }

    pub fn runFrame(self: *Emulator) void {
        const starting_cycles: u32 = @as(u32, self.timer.counter);
        var cycles = 0;
        while (cycles < hw.Timings.cyclesPerFrame - self._frame_sync) {
            self.cpu.step();
            cycles = self.timer.counter -% starting_cycles;
        }

        self._frame_sync = cycles - hw.Timings.cyclesPerFrame;
    }

    pub fn deinit(self: *Emulator) void {
        self._arena.deinit();
    }
};
