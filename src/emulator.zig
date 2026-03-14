const std = @import("std");
const Cpu = @import("cpu.zig").Cpu;
const Mmu = @import("mmu.zig").Mmu;
const Mbc = @import("mbc.zig").Mbc;

pub const Emulator = struct {
    _arena: std.heap.ArenaAllocator,
    _cpu: *Cpu,
    _mbc: *Mbc,

    pub fn init(allocator: std.mem.Allocator, rom_buf: []const u8) !*Emulator {
        var arena = std.heap.ArenaAllocator.init(allocator);
        errdefer arena.deinit();
        const aa = arena.allocator();

        const rom = try aa.dupe(u8, rom_buf); // CACHE LOCALITY LETS FUCKING GOOOOOO
        const mbc = try Mbc.init(aa, rom);
        const emu = try aa.create(Emulator);

        emu._cpu = try aa.create(Cpu);
        emu._cpu.* = .{
            .memory = .{
                .mbc = mbc,
            },
        };

        emu._arena = arena;
        emu._mbc = &emu._cpu.memory.mbc;

        return emu;
    }

    pub fn runFrame(self: *Emulator) void {
        _ = self;
    }

    pub fn deinit(self: *Emulator) !void {
        try self._arena.deinit();
    }
};
