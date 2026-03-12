const std = @import("std");
const CPU = @import("cpu.zig").CPU;
const MBC = @import("mbc.zig").MBC;

pub const Emulator = struct {
    arena: std.heap.ArenaAllocator,
    cpu: *CPU,
    mbc: *MBC,
    isRunning: bool,

    pub fn init(arena: ArenaAllocator, rom_buf: []u8) !*Emulator {
        
    };
};
