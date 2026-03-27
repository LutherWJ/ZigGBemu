// Exposes the emulator interface to the C ABI
// Can be used for WASM as well.
const std = @import("std");
const Emulator = @import("emulator.zig").Emulator;
const Cpu = @import("cpu").Cpu;

var emu_instance: ?*Emulator = null;
const allocator = std.heap.page_allocator;

export fn init_emulator(rom_ptr: [*]const u8, rom_size: usize) i32 {
    const rom = rom_ptr[0..rom_size];
    emu_instance = Emulator.init(allocator, rom) catch return 1;
    return 0;
}

export fn run_frame() void {
    if (emu_instance) |e| e.runFrame();
}

export fn deinit_emulator() void {
    if (emu_instance) |e| {
        e.deinit();
        emu_instance = null;
    }
}

export fn step() void {
    if (emu_instance) |e| e.cpu.step();
}

// Expose registers
export fn get_reg_a() u8 {
    return if (emu_instance) |e| e.cpu.a else 0;
}
export fn get_reg_f() u8 {
    return if (emu_instance) |e| e.cpu.f else 0;
}
export fn get_reg_b() u8 {
    return if (emu_instance) |e| e.cpu.b else 0;
}
export fn get_reg_c() u8 {
    return if (emu_instance) |e| e.cpu.c else 0;
}
export fn get_reg_d() u8 {
    return if (emu_instance) |e| e.cpu.d else 0;
}
export fn get_reg_e() u8 {
    return if (emu_instance) |e| e.cpu.e else 0;
}
export fn get_reg_h() u8 {
    return if (emu_instance) |e| e.cpu.h else 0;
}
export fn get_reg_l() u8 {
    return if (emu_instance) |e| e.cpu.l else 0;
}
export fn get_reg_pc() u16 {
    return if (emu_instance) |e| e.cpu.pc else 0;
}
export fn get_reg_sp() u16 {
    return if (emu_instance) |e| e.cpu.sp else 0;
}

// Expose memory
export fn get_vram_ptr() [*]u8 {
    return if (emu_instance) |e| @ptrCast(&e.mmu.vram) else undefined;
}
export fn get_wram_ptr() [*]u8 {
    return if (emu_instance) |e| @ptrCast(&e.mmu.wram0) else undefined;
}
export fn get_hram_ptr() [*]u8 {
    return if (emu_instance) |e| @ptrCast(&e.mmu.hram) else undefined;
}

// Expose serial output
export fn get_serial_len() usize {
    return if (emu_instance) |e| e.sdt.fifo.readableLength() else 0;
}

export fn read_serial_byte() u8 {
    return if (emu_instance) |e| e.sdt.fifo.readItem() orelse 0 else 0;
}

// Interrupt state
export fn get_ie() u8 {
    return if (emu_instance) |e| e.interrupts.ie else 0;
}
export fn get_if() u8 {
    return if (emu_instance) |e| e.interrupts.ifr else 0;
}

// Whatever else
export fn get_cpu_ptr() ?*Cpu {
    return if (emu_instance) |e| e.cpu else null;
}
