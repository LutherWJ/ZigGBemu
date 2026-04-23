// Exposes the emulator interface to the C ABI
// Can be used for WASM as well.
const std = @import("std");
const builtin = @import("builtin");
const Emulator = @import("emulator.zig").Emulator;
const Cpu = @import("cpu").Cpu;

var emu_instance: ?*Emulator = null;

// Pre-allocate a 32MB heap buffer inside the WASM memory space
var heap_buffer: [32 * 1024 * 1024]u8 = undefined;
var fba = std.heap.FixedBufferAllocator.init(&heap_buffer);
const allocator = if (builtin.os.tag == .freestanding) fba.allocator() else std.heap.page_allocator;

// Dedicated 8MB area for ROM loading
export var rom_buffer: [8 * 1024 * 1024]u8 = undefined;

export fn get_rom_buffer_ptr() [*]u8 {
    return &rom_buffer;
}

export fn init_emulator(rom_size: usize) i32 {
    if (emu_instance) |e| {
        e.deinit();
        emu_instance = null;
    }

    if (rom_size > rom_buffer.len) return 1;
    if (rom_size < 0x0150) return 2; // ROM too small for header

    const rom = rom_buffer[0..rom_size];
    emu_instance = Emulator.init(allocator, rom) catch |err| {
        if (err == error.UnsupportedMbcType) return 3;
        if (err == error.OutOfMemory) return 4;
        return 5;
    };
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

// Expose display
export fn get_frame_buffer_ptr() [*]u32 {
    return if (emu_instance) |e| @ptrCast(&e.ppu.frame_buf) else undefined;
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
    return if (emu_instance) |e| @ptrCast(&e.ppu.vram) else undefined;
}
export fn get_wram0_ptr() [*]u8 {
    return if (emu_instance) |e| @ptrCast(&e.mmu.wram0) else undefined;
}
export fn get_wram1_ptr() [*]u8 {
    return if (emu_instance) |e| @ptrCast(&e.mmu.wram1) else undefined;
}
export fn get_hram_ptr() [*]u8 {
    return if (emu_instance) |e| @ptrCast(&e.mmu.hram) else undefined;
}
export fn get_oam_ptr() [*]u8 {
    return if (emu_instance) |e| @ptrCast(&e.ppu.oam) else undefined;
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
