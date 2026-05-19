// Exposes the emulator interface to the C ABI
const std = @import("std");
const builtin = @import("builtin");
const Emulator = @import("emulator").Emulator;
const Cpu = @import("cpu").Cpu;
const joypad = @import("joypad");

// Logging
extern fn js_log_write(ptr: [*]const u8, len: usize) void;
extern fn js_panic(ptr: [*]const u8, len: usize) void;

pub fn log(msg: []const u8) void {
    js_log_write(msg.ptr, msg.len);
}

pub fn panic(msg: []const u8, stack_trace: ?*std.builtin.StackTrace, addr: ?usize) noreturn {
    var buf: [2048]u8 = undefined;
    var fbs = std.io.fixedBufferStream(&buf);
    const writer = fbs.writer();

    writer.print("--- ZIG CRASH REPORT ---\n", .{}) catch {};
    
    // Check if this looks like an assertion failure
    if (std.mem.indexOf(u8, msg, "assertion failed") != null or std.mem.indexOf(u8, msg, "reached unreachable code") != null) {
        writer.print("CRITICAL: {s}\n", .{msg}) catch {};
    } else {
        writer.print("Message: {s}\n", .{msg}) catch {};
    }
    
    if (addr) |a| {
        writer.print("Fault Address (WASM): 0x{X}\n", .{a}) catch {};
    }

    if (emu_instance) |e| {
        writer.print("\n[Emulator State]\n", .{}) catch {};
        writer.print("  PC: 0x{X:0>4}\n", .{e.cpu.pc}) catch {};
        writer.print("  SP: 0x{X:0>4}\n", .{e.cpu.sp}) catch {};
        writer.print("  Regs: A:{X:0>2} F:{X:0>2} B:{X:0>2} C:{X:0>2} D:{X:0>2} E:{X:0>2} H:{X:0>2} L:{X:0>2}\n", .{
            e.cpu.a, e.cpu.f, e.cpu.b, e.cpu.c, e.cpu.d, e.cpu.e, e.cpu.h, e.cpu.l,
        }) catch {};
        writer.print("  LY: {d}  Mode: {s}\n", .{e.ppu.ly, @tagName(e.ppu.state)}) catch {};
    }

    if (stack_trace) |st| {
        writer.print("\n[Stack Trace - Instruction Addresses]\n", .{}) catch {};
        for (st.instruction_addresses[0..st.index]) |address| {
            writer.print("  0x{X}\n", .{address}) catch {};
        }
    }

    const final_msg = fbs.getWritten();
    js_panic(final_msg.ptr, final_msg.len);
    @trap();
}

var emu_instance: ?*Emulator = null;

// Pre-allocate a 32MB heap buffer inside the WASM memory space
var heap_buffer: [32 * 1024 * 1024]u8 align(4096) = undefined;
var fba = std.heap.FixedBufferAllocator.init(&heap_buffer);
const allocator = if (builtin.os.tag == .freestanding) fba.allocator() else std.heap.page_allocator;

// Dedicated 8MB area for ROM loading
export var rom_buffer: [8 * 1024 * 1024]u8 align(4096) = undefined;

export fn get_rom_buffer_ptr() [*]u8 {
    return &rom_buffer;
}

export fn init_emulator(rom_size: usize) i32 {
    log("INFO: init_emulator called");
    if (emu_instance) |e| {
        e.deinit();
        emu_instance = null;
    }

    // Reset the allocator for a clean start
    fba.reset();

    if (rom_size > rom_buffer.len) return 1;
    if (rom_size < 0x0150) return 2; // ROM too small for header

    const rom = rom_buffer[0..rom_size];
    emu_instance = Emulator.init(allocator, rom) catch |err| {
        log("ERROR: Emulator initialization failed");
        if (err == error.UnsupportedMbcType) return 3;
        if (err == error.OutOfMemory) return 4;
        return 5;
    };
    log("INFO: Emulator initialized successfully");
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

export fn set_button(button_id: u8, pressed: bool) void {
    if (emu_instance) |e| {
        const btn: joypad.Button = switch (button_id) {
            0 => .a,
            1 => .b,
            2 => .select,
            3 => .start,
            4 => .right,
            5 => .left,
            6 => .up,
            7 => .down,
            else => return,
        };
        e.joypad.setButton(btn, pressed);
    }
}

// Expose display
export fn get_frame_buffer_ptr() [*]u32 {
    return if (emu_instance) |e| @ptrCast(&e.ppu.frame_buf.buffer) else undefined;
}

// Expose registers
export fn get_clock() u16 {
    return if (emu_instance) |e| e.timer.counter else 0;
}
export fn get_joypad_state() u8 {
    if (emu_instance) |e| {
        var state: u8 = 0;
        if (e.joypad.a) state |= (1 << 0);
        if (e.joypad.b) state |= (1 << 1);
        if (e.joypad.select) state |= (1 << 2);
        if (e.joypad.start) state |= (1 << 3);
        if (e.joypad.right) state |= (1 << 4);
        if (e.joypad.left) state |= (1 << 5);
        if (e.joypad.up) state |= (1 << 6);
        if (e.joypad.down) state |= (1 << 7);
        return state;
    }
    return 0;
}
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
// Expose PPU registers
export fn get_ppu_lcdc() u8 {
    return if (emu_instance) |e| @bitCast(e.ppu.lcdc) else 0;
}
export fn get_ppu_stat() u8 {
    return if (emu_instance) |e| @bitCast(e.ppu.stat) else 0;
}
export fn get_ppu_scy() u8 {
    return if (emu_instance) |e| e.ppu.scy else 0;
}
export fn get_ppu_scx() u8 {
    return if (emu_instance) |e| e.ppu.scx else 0;
}
export fn get_ppu_ly() u8 {
    return if (emu_instance) |e| e.ppu.ly else 0;
}
export fn get_ppu_lyc() u8 {
    return if (emu_instance) |e| e.ppu.lyc else 0;
}
export fn get_ppu_wx() u8 {
    return if (emu_instance) |e| e.ppu.wx else 0;
}
export fn get_ppu_wy() u8 {
    return if (emu_instance) |e| e.ppu.wy else 0;
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
export fn get_ie() u8 {
    return if (emu_instance) |e| e.interrupts.ie else 0;
}
export fn get_if() u8 {
    return if (emu_instance) |e| e.interrupts.ifr else 0;
}
export fn get_oam_ptr() [*]u8 {
    return if (emu_instance) |e| @ptrCast(&e.ppu.oam) else undefined;
}

// SRAM Management
export fn get_sram_ptr() ?[*]u8 {
    return if (emu_instance) |e| e.mmu.mbc.getRamPtr() else null;
}
export fn get_sram_size() usize {
    return if (emu_instance) |e| e.mmu.mbc.getRamSize() else 0;
}
export fn is_sram_dirty() bool {
    return if (emu_instance) |e| e.mmu.mbc.isDirty() else false;
}
export fn clear_sram_dirty() void {
    if (emu_instance) |e| e.mmu.mbc.clearDirty();
}

// Whatever else
export fn ping() void {
    log("PONG: Connection to Zig confirmed.");
}
