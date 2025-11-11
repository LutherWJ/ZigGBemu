const std = @import("std");
const Memory = @import("memory.zig").Memory;
const InterruptBit = @import("memory.zig").InterruptBit;

const InstructionFn = *const fn (*CPU) void;
const Flag = enum(u3) { z = 7, n = 6, h = 5, c = 4 };
const Register = enum { a, f, b, c, d, e, h, l };
const U16Register = enum { bc, de, hl, af, sp, pc };
const Conditions = enum { always, nz, z, nc, c }; // Used for conditional instructions like CALL or JP
const ImeState = enum { disabled, enable_pending, enabled }; // interrupts are sometimes delayed one cycle
const SourceType = enum { register, hl_ptr, immediate };

// Helper function to fetch operand value based on source type
fn fetchSource(cpu: *CPU, comptime src_type: SourceType, comptime reg: ?Register) u8 {
    return switch (src_type) {
        .register => cpu.getRegister(reg.?).*,
        .hl_ptr => cpu.memory.read(cpu.getU16Register(.hl), u8),
        .immediate => blk: {
            const val = cpu.memory.read(cpu.pc, u8);
            cpu.pc += 1;
            break :blk val;
        },
    };
}

// Comptime instruction generators
fn makeLdRegReg(comptime dst: Register, comptime src: Register) InstructionFn {
    return struct {
        fn f(cpu: *CPU) void {
            cpu.getRegister(dst).* = cpu.getRegister(src).*;
        }
    }.f;
}

fn makeLdRegHlPtr(comptime dst: Register) InstructionFn {
    return struct {
        fn f(cpu: *CPU) void {
            cpu.getRegister(dst).* = cpu.memory.read(cpu.getU16Register(.hl), u8);
        }
    }.f;
}

fn makeLdRegU16Ptr(comptime dst: Register, comptime src: U16Register) InstructionFn {
    return struct {
        fn f(cpu: *CPU) void {
            const address = cpu.getU16Register(src);
            cpu.getRegister(dst).* = cpu.memory.read(address, u8);
        }
    }.f;
}

fn makeLdU16PtrReg(comptime dst: U16Register, comptime src: Register) InstructionFn {
    return struct {
        fn f(cpu: *CPU) void {
            const address = cpu.getU16Register(dst);
            cpu.memory.write(address, cpu.getRegister(src).*);
        }
    }.f;
}

fn makeLdRegD8(comptime dst: Register) InstructionFn {
    return struct {
        fn f(cpu: *CPU) void {
            cpu.getRegister(dst).* = cpu.memory.read(cpu.pc, u8);
            cpu.pc += 1;
        }
    }.f;
}

fn makeLdU16RegD16(comptime dst: U16Register) InstructionFn {
    return struct {
        fn f(cpu: *CPU) void {
            const d16 = cpu.memory.read(cpu.pc, u16);
            cpu.pc += 2;
            cpu.setU16Register(dst, d16);
        }
    }.f;
}

fn makeLdHlPtrReg(comptime src: Register) InstructionFn {
    return struct {
        fn f(cpu: *CPU) void {
            cpu.memory.write(cpu.getU16Register(.hl), cpu.getRegister(src).*);
        }
    }.f;
}

fn makeAdd(comptime src_type: SourceType, comptime reg: ?Register) InstructionFn {
    return struct {
        fn f(cpu: *CPU) void {
            const old = cpu.a;
            const value = fetchSource(cpu, src_type, reg);
            cpu.a +%= value;
            cpu.updateMathFlags(u8, old, value, cpu.a, false);
        }
    }.f;
}

fn makeSub(comptime src_type: SourceType, comptime reg: ?Register) InstructionFn {
    return struct {
        fn f(cpu: *CPU) void {
            const old = cpu.a;
            const value = fetchSource(cpu, src_type, reg);
            cpu.a -%= value;
            cpu.updateMathFlags(u8, old, value, cpu.a, true);
        }
    }.f;
}

fn makeAnd(comptime src_type: SourceType, comptime reg: ?Register) InstructionFn {
    return struct {
        fn f(cpu: *CPU) void {
            const value = fetchSource(cpu, src_type, reg);
            cpu.a &= value;
            cpu.checkFlag(.z, cpu.a == 0);
            cpu.unsetFlag(.n);
            cpu.setFlag(.h);
            cpu.unsetFlag(.c);
        }
    }.f;
}

fn makeOr(comptime src_type: SourceType, comptime reg: ?Register) InstructionFn {
    return struct {
        fn f(cpu: *CPU) void {
            const value = fetchSource(cpu, src_type, reg);
            cpu.a |= value;
            cpu.checkFlag(.z, cpu.a == 0);
            cpu.unsetFlag(.n);
            cpu.unsetFlag(.h);
            cpu.unsetFlag(.c);
        }
    }.f;
}

fn makeXor(comptime src_type: SourceType, comptime reg: ?Register) InstructionFn {
    return struct {
        fn f(cpu: *CPU) void {
            const value = fetchSource(cpu, src_type, reg);
            cpu.a ^= value;
            cpu.checkFlag(.z, cpu.a == 0);
            cpu.unsetFlag(.n);
            cpu.unsetFlag(.h);
            cpu.unsetFlag(.c);
        }
    }.f;
}

fn makeCp(comptime src_type: SourceType, comptime reg: ?Register) InstructionFn {
    return struct {
        fn f(cpu: *CPU) void {
            const value = fetchSource(cpu, src_type, reg);
            const result = cpu.a -% value;
            cpu.checkFlag(.z, result == 0);
            cpu.setFlag(.n);
            cpu.checkFlag(.h, (cpu.a & 0x0F) < (value & 0x0F));
            cpu.checkFlag(.c, cpu.a < value);
        }
    }.f;
}

fn makeIncReg(comptime dst: Register) InstructionFn {
    return struct {
        fn f(cpu: *CPU) void {
            const reg = cpu.getRegister(dst);
            const old = reg.*;
            reg.* +%= 1;
            cpu.updateIncFlags(old, reg.*, false);
        }
    }.f;
}

fn makeDecReg(comptime dst: Register) InstructionFn {
    return struct {
        fn f(cpu: *CPU) void {
            const reg = cpu.getRegister(dst);
            const old = reg.*;
            reg.* -%= 1;
            cpu.updateIncFlags(old, reg.*, true);
        }
    }.f;
}

fn makeIncU16Reg(comptime dst: U16Register) InstructionFn {
    return struct {
        fn f(cpu: *CPU) void {
            cpu.setU16Register(dst, cpu.getU16Register(dst) +% 1);
        }
    }.f;
}

fn makeDecU16Reg(comptime dst: U16Register) InstructionFn {
    return struct {
        fn f(cpu: *CPU) void {
            cpu.setU16Register(dst, cpu.getU16Register(dst) -% 1);
        }
    }.f;
}

fn makeAddHl(comptime src: U16Register) InstructionFn {
    return struct {
        fn f(cpu: *CPU) void {
            const hl = cpu.getU16Register(.hl);
            const value = cpu.getU16Register(src);
            const result = hl +% value;
            cpu.updateMathFlags(u16, hl, value, result, false);
            cpu.setU16Register(.hl, result);
        }
    }.f;
}

fn makeAdc(comptime src_type: SourceType, comptime reg: ?Register) InstructionFn {
    return struct {
        fn f(cpu: *CPU) void {
            const old = cpu.a;
            const carry: u8 = @intFromBool(cpu.readFlag(.c));
            const value = fetchSource(cpu, src_type, reg);
            cpu.a +%= value +% carry;
            cpu.updateMathFlags(u8, old, value +% carry, cpu.a, false);
        }
    }.f;
}

fn makeSbc(comptime src_type: SourceType, comptime reg: ?Register) InstructionFn {
    return struct {
        fn f(cpu: *CPU) void {
            const old = cpu.a;
            const carry: u8 = @intFromBool(cpu.readFlag(.c));
            const value = fetchSource(cpu, src_type, reg);
            cpu.a -%= value +% carry;
            cpu.updateMathFlags(u8, old, value +% carry, cpu.a, true);
        }
    }.f;
}

fn makeJumpRelative(comptime cond: Conditions) InstructionFn {
    return struct {
        fn f(cpu: *CPU) void {
            const offset: i8 = @bitCast(cpu.memory.read(cpu.pc, u8));
            cpu.pc += 1;

            const should_jump = switch (cond) {
                .always => true,
                .nz => (cpu.f & 0b10000000) != 0b10000000,
                .z => (cpu.f & 0b10000000) == 0b10000000,
                .nc => (cpu.f & 0b00010000) != 0b00010000,
                .c => (cpu.f & 0b00010000) == 0b00010000,
            };

            if (should_jump) {
                const offset_i16: i16 = offset;
                const offset_u16: u16 = @bitCast(offset_i16);
                cpu.pc +%= offset_u16;
            }
        }
    }.f;
}

fn makeCall(comptime cond: Conditions) InstructionFn {
    return struct {
        fn f(cpu: *CPU) void {
            const address: u16 = cpu.memory.read(cpu.pc, u16);
            cpu.pc +%= 2;

            const should_call = switch (cond) {
                .always => true,
                .nz => (cpu.f & 0b10000000) != 0b10000000,
                .z => (cpu.f & 0b10000000) == 0b10000000,
                .nc => (cpu.f & 0b00010000) != 0b00010000,
                .c => (cpu.f & 0b00010000) == 0b00010000,
            };

            if (should_call) {
                cpu.pushU16(cpu.pc);
                cpu.pc = address;
            }
        }
    }.f;
}

fn makeRet(comptime cond: Conditions) InstructionFn {
    return struct {
        fn f(cpu: *CPU) void {
            const should_call = switch (cond) {
                .always => true,
                .nz => (cpu.f & 0b10000000) != 0b10000000,
                .z => (cpu.f & 0b10000000) == 0b10000000,
                .nc => (cpu.f & 0b00010000) != 0b00010000,
                .c => (cpu.f & 0b00010000) == 0b00010000,
            };

            if (should_call) {
                cpu.pc = cpu.popU16();
            }
        }
    }.f;
}

fn makePush(comptime pair: U16Register) InstructionFn {
    return struct {
        fn f(cpu: *CPU) void {
            const value = cpu.getU16Register(pair);
            cpu.sp -%= 2;
            cpu.memory.write(cpu.sp, value);
        }
    }.f;
}

fn makePop(comptime pair: U16Register) InstructionFn {
    return struct {
        fn f(cpu: *CPU) void {
            const value = cpu.memory.read(cpu.sp, u16);
            cpu.sp +%= 2;
            cpu.setU16Register(pair, value);
        }
    }.f;
}

fn makeRST(comptime opcode: u8) InstructionFn {
    return struct {
        fn f(cpu: *CPU) void {
            cpu.pushU16(cpu.pc);
            cpu.pc = opcode & 0b00111000; // black magic
        }
    }.f;
}

// Instruction table matching opcodes to their corresponding functions
const instruction_table = blk: {
    var table = [_]InstructionFn{CPU.unknown_opcode} ** 256;
    table[0x00] = CPU.nop;
    table[0x01] = makeLdU16RegD16(.bc);
    table[0x02] = makeLdU16PtrReg(.bc, .a);
    table[0x03] = makeIncU16Reg(.bc);
    table[0x04] = makeIncReg(.b);
    table[0x05] = makeDecReg(.b);
    table[0x06] = makeLdRegD8(.b);
    table[0x07] = CPU.rlca;
    table[0x08] = CPU.ld_a16_ptr_sp;
    table[0x09] = makeAddHl(.bc);
    table[0x0A] = makeLdRegU16Ptr(.a, .bc);
    table[0x0B] = makeDecU16Reg(.bc);
    table[0x0C] = makeIncReg(.c);
    table[0x0D] = makeDecReg(.c);
    table[0x0E] = makeLdRegD8(.c);
    table[0x0F] = CPU.rrca;
    table[0x11] = makeLdU16RegD16(.de);
    table[0x12] = makeLdU16PtrReg(.de, .a);
    table[0x13] = makeIncU16Reg(.de);
    table[0x14] = makeIncReg(.d);
    table[0x15] = makeDecReg(.d);
    table[0x16] = makeLdRegD8(.d);
    table[0x17] = CPU.rla;
    table[0x19] = makeAddHl(.de);
    table[0x1A] = makeLdRegU16Ptr(.a, .de);
    table[0x1B] = makeDecU16Reg(.de);
    table[0x1C] = makeIncReg(.e);
    table[0x1D] = makeDecReg(.e);
    table[0x1E] = makeLdRegD8(.e);
    table[0x1F] = CPU.rra;
    table[0x18] = makeJumpRelative(.always);
    table[0x20] = makeJumpRelative(.nz);
    table[0x21] = makeLdU16RegD16(.hl);
    table[0x22] = CPU.ld_hl_ptr_plus_a;
    table[0x23] = makeIncU16Reg(.hl);
    table[0x24] = makeIncReg(.h);
    table[0x25] = makeDecReg(.h);
    table[0x26] = makeLdRegD8(.h);
    table[0x28] = makeJumpRelative(.z);
    table[0x29] = makeAddHl(.hl);
    table[0x2A] = CPU.ld_a_hl_ptr_plus;
    table[0x2B] = makeDecU16Reg(.hl);
    table[0x2C] = makeIncReg(.l);
    table[0x2D] = makeDecReg(.l);
    table[0x2E] = makeLdRegD8(.l);
    table[0x30] = makeJumpRelative(.nc);
    table[0x32] = CPU.ld_hl_ptr_minus_a;
    table[0x33] = makeIncU16Reg(.sp);
    table[0x34] = CPU.inc_hl_ptr;
    table[0x35] = CPU.dec_hl_ptr;
    table[0x36] = CPU.ld_hl_ptr_d8;
    table[0x38] = makeJumpRelative(.c);
    table[0x39] = makeAddHl(.sp);
    table[0x3A] = CPU.ld_a_hl_ptr_minus;
    table[0x3B] = makeDecU16Reg(.sp);
    table[0x3C] = makeIncReg(.a);
    table[0x3D] = makeDecReg(.a);
    table[0x3E] = makeLdRegD8(.a);
    table[0x40] = makeLdRegReg(.b, .b);
    table[0x41] = makeLdRegReg(.b, .c);
    table[0x42] = makeLdRegReg(.b, .d);
    table[0x43] = makeLdRegReg(.b, .e);
    table[0x44] = makeLdRegReg(.b, .h);
    table[0x45] = makeLdRegReg(.b, .l);
    table[0x46] = makeLdRegHlPtr(.b);
    table[0x47] = makeLdRegReg(.b, .a);
    table[0x48] = makeLdRegReg(.c, .b);
    table[0x49] = makeLdRegReg(.c, .c);
    table[0x4A] = makeLdRegReg(.c, .d);
    table[0x4B] = makeLdRegReg(.c, .e);
    table[0x4C] = makeLdRegReg(.c, .h);
    table[0x4D] = makeLdRegReg(.c, .l);
    table[0x4E] = makeLdRegHlPtr(.c);
    table[0x4F] = makeLdRegReg(.c, .a);
    table[0x50] = makeLdRegReg(.d, .b);
    table[0x51] = makeLdRegReg(.d, .c);
    table[0x52] = makeLdRegReg(.d, .d);
    table[0x53] = makeLdRegReg(.d, .e);
    table[0x54] = makeLdRegReg(.d, .h);
    table[0x55] = makeLdRegReg(.d, .l);
    table[0x56] = makeLdRegHlPtr(.d);
    table[0x57] = makeLdRegReg(.d, .a);
    table[0x58] = makeLdRegReg(.e, .b);
    table[0x59] = makeLdRegReg(.e, .c);
    table[0x5A] = makeLdRegReg(.e, .d);
    table[0x5B] = makeLdRegReg(.e, .e);
    table[0x5C] = makeLdRegReg(.e, .h);
    table[0x5D] = makeLdRegReg(.e, .l);
    table[0x5E] = makeLdRegHlPtr(.e);
    table[0x5F] = makeLdRegReg(.e, .a);
    table[0x60] = makeLdRegReg(.h, .b);
    table[0x61] = makeLdRegReg(.h, .c);
    table[0x62] = makeLdRegReg(.h, .d);
    table[0x63] = makeLdRegReg(.h, .e);
    table[0x64] = makeLdRegReg(.h, .h);
    table[0x65] = makeLdRegReg(.h, .l);
    table[0x66] = makeLdRegHlPtr(.h);
    table[0x67] = makeLdRegReg(.h, .a);
    table[0x68] = makeLdRegReg(.l, .b);
    table[0x69] = makeLdRegReg(.l, .c);
    table[0x6A] = makeLdRegReg(.l, .d);
    table[0x6B] = makeLdRegReg(.l, .e);
    table[0x6C] = makeLdRegReg(.l, .h);
    table[0x6D] = makeLdRegReg(.l, .l);
    table[0x6E] = makeLdRegHlPtr(.l);
    table[0x6F] = makeLdRegReg(.l, .a);
    table[0x70] = makeLdHlPtrReg(.b);
    table[0x71] = makeLdHlPtrReg(.c);
    table[0x72] = makeLdHlPtrReg(.d);
    table[0x73] = makeLdHlPtrReg(.e);
    table[0x74] = makeLdHlPtrReg(.h);
    table[0x75] = makeLdHlPtrReg(.l);
    table[0x76] = CPU.halt;
    table[0x77] = makeLdHlPtrReg(.a);
    table[0x78] = makeLdRegReg(.a, .b);
    table[0x79] = makeLdRegReg(.a, .c);
    table[0x7A] = makeLdRegReg(.a, .d);
    table[0x7B] = makeLdRegReg(.a, .e);
    table[0x7C] = makeLdRegReg(.a, .h);
    table[0x7D] = makeLdRegReg(.a, .l);
    table[0x7E] = makeLdRegHlPtr(.a);
    table[0x7F] = makeLdRegReg(.a, .a);
    table[0x80] = makeAdd(.register, .b);
    table[0x81] = makeAdd(.register, .c);
    table[0x82] = makeAdd(.register, .d);
    table[0x83] = makeAdd(.register, .e);
    table[0x84] = makeAdd(.register, .h);
    table[0x85] = makeAdd(.register, .l);
    table[0x86] = makeAdd(.hl_ptr, null);
    table[0x87] = makeAdd(.register, .a);
    table[0x88] = makeAdc(.register, .b);
    table[0x89] = makeAdc(.register, .c);
    table[0x8A] = makeAdc(.register, .d);
    table[0x8B] = makeAdc(.register, .e);
    table[0x8C] = makeAdc(.register, .h);
    table[0x8D] = makeAdc(.register, .l);
    table[0x8E] = makeAdc(.hl_ptr, null);
    table[0x8F] = makeAdc(.register, .a);
    table[0x90] = makeSub(.register, .b);
    table[0x91] = makeSub(.register, .c);
    table[0x92] = makeSub(.register, .d);
    table[0x93] = makeSub(.register, .e);
    table[0x94] = makeSub(.register, .h);
    table[0x95] = makeSub(.register, .l);
    table[0x96] = makeSub(.hl_ptr, null);
    table[0x97] = makeSub(.register, .a);
    table[0x98] = makeSbc(.register, .b);
    table[0x99] = makeSbc(.register, .c);
    table[0x9A] = makeSbc(.register, .d);
    table[0x9B] = makeSbc(.register, .e);
    table[0x9C] = makeSbc(.register, .h);
    table[0x9D] = makeSbc(.register, .l);
    table[0x9E] = makeSbc(.hl_ptr, null);
    table[0x9F] = makeSbc(.register, .a);
    table[0xA0] = makeAnd(.register, .b);
    table[0xA1] = makeAnd(.register, .c);
    table[0xA2] = makeAnd(.register, .d);
    table[0xA3] = makeAnd(.register, .e);
    table[0xA4] = makeAnd(.register, .h);
    table[0xA5] = makeAnd(.register, .l);
    table[0xA6] = makeAnd(.hl_ptr, null);
    table[0xA7] = makeAnd(.register, .a);
    table[0xA8] = makeXor(.register, .b);
    table[0xA9] = makeXor(.register, .c);
    table[0xAA] = makeXor(.register, .d);
    table[0xAB] = makeXor(.register, .e);
    table[0xAC] = makeXor(.register, .h);
    table[0xAD] = makeXor(.register, .l);
    table[0xAE] = makeXor(.hl_ptr, null);
    table[0xAF] = makeXor(.register, .a);
    table[0xB0] = makeOr(.register, .b);
    table[0xB1] = makeOr(.register, .c);
    table[0xB2] = makeOr(.register, .d);
    table[0xB3] = makeOr(.register, .e);
    table[0xB4] = makeOr(.register, .h);
    table[0xB5] = makeOr(.register, .l);
    table[0xB6] = makeOr(.hl_ptr, null);
    table[0xB7] = makeOr(.register, .a);
    table[0xB8] = makeCp(.register, .b);
    table[0xB9] = makeCp(.register, .c);
    table[0xBA] = makeCp(.register, .d);
    table[0xBB] = makeCp(.register, .e);
    table[0xBC] = makeCp(.register, .h);
    table[0xBD] = makeCp(.register, .l);
    table[0xBE] = makeCp(.hl_ptr, null);
    table[0xBF] = makeCp(.register, .a);
    table[0xC0] = makeRet(.nz);
    table[0xC1] = makePop(.bc);
    table[0xC4] = makeCall(.nz);
    table[0xC5] = makePush(.bc);
    table[0xC6] = makeAdd(.immediate, null);
    table[0xC7] = makeRST(0xC7);
    table[0xC8] = makeRet(.z);
    table[0xC9] = makeRet(.always);
    table[0xCC] = makeCall(.z);
    table[0xCD] = makeCall(.always);
    table[0xCE] = makeAdc(.immediate, null);
    table[0xCF] = makeRST(0xCF);
    table[0xD0] = makeRet(.nc);
    table[0xD1] = makePop(.de);
    table[0xD4] = makeCall(.nc);
    table[0xD5] = makePush(.de);
    table[0xD6] = makeSub(.immediate, null);
    table[0xD7] = makeRST(0xD7);
    table[0xD8] = makeRet(.c);
    table[0xD9] = CPU.reti;
    table[0xDC] = makeCall(.c);
    table[0xDE] = makeSbc(.immediate, null);
    table[0xDF] = makeRST(0xDF);
    table[0xE1] = makePop(.hl);
    table[0xE5] = makePush(.hl);
    table[0xE6] = makeAnd(.immediate, null);
    table[0xE7] = makeRST(0xE7);
    table[0xEE] = makeXor(.immediate, null);
    table[0xEF] = makeRST(0xEF);
    table[0xF1] = makePop(.af);
    table[0xF3] = CPU.di;
    table[0xF5] = makePush(.af);
    table[0xF6] = makeOr(.immediate, null);
    table[0xF7] = makeRST(0xF7);
    table[0xFB] = CPU.ei;
    table[0xFE] = makeCp(.immediate, null);
    table[0xFF] = makeRST(0xFF);
    break :blk table;
};

pub const CPU = struct {
    a: u8 = 0, // accumulator
    f: u8 = 0, // flags
    b: u8 = 0,
    c: u8 = 0,
    d: u8 = 0,
    e: u8 = 0,
    h: u8 = 0,
    l: u8 = 0,

    sp: u16 = 0, // stack pointer
    pc: u16 = 0, // program counter

    memory: Memory = .{},

    ime_state: ImeState = .disabled,
    halted: bool = false,
    haltBug: bool = false,

    pub fn step(self: *CPU) !void {
        // Handle pending IME enable
        if (self.ime_state == .enable_pending) {
            self.ime_state = .enabled;
        }

        if (self.memory.getPendingInterrupt()) |interrupt| {
            self.handleInterrupt(interrupt);
            return;
        }
        const opcode = self.memory.read(self.pc, u8);

        // Implementation of HALT hardware bug
        if (self.haltBug) {
            self.haltBug = false;
        } else {
            self.pc += 1;
        }

        instruction_table[opcode](self);
    }

    // Fast boot implementation. Initializes memory to the post boot state and does nothing more.
    pub fn boot(self: *CPU) void {
        self.a = 0x11;
        self.f = 0xB0;
        self.c = 0x13;
        self.e = 0xD8;
        self.h = 0x01;
        self.l = 0x4D;
        self.sp = 0x0100;

        self.memory.write(0xFF05, 0x00);
        self.memory.write(0xFF06, 0x00);
        self.memory.write(0xFF07, 0x00);
        self.memory.write(0xFF10, 0x80);
        self.memory.write(0xFF11, 0xBF);
        self.memory.write(0xFF12, 0xF3);
        self.memory.write(0xFF14, 0xBF);
        self.memory.write(0xFF16, 0x3F);
        self.memory.write(0xFF17, 0x00);
        self.memory.write(0xFF19, 0xBF);
        self.memory.write(0xFF1A, 0x7F);
        self.memory.write(0xFF1B, 0xFF);
        self.memory.write(0xFF1C, 0x9F);
        self.memory.write(0xFF1E, 0xBF);
        self.memory.write(0xFF20, 0xFF);
        self.memory.write(0xFF21, 0x00);
        self.memory.write(0xFF22, 0x00);
        self.memory.write(0xFF23, 0xBF);
        self.memory.write(0xFF24, 0x77);
        self.memory.write(0xFF25, 0xF3);
        self.memory.write(0xFF26, 0xF1);
        self.memory.write(0xFF40, 0x91);
        self.memory.write(0xFF42, 0x00);
        self.memory.write(0xFF43, 0x00);
        self.memory.write(0xFF45, 0x00);
        self.memory.write(0xFF47, 0xFC);
        self.memory.write(0xFF48, 0xFF);
        self.memory.write(0xFF49, 0xFF);
        self.memory.write(0xFF4A, 0x00);
        self.memory.write(0xFF4B, 0x00);
        self.memory.write(0xFFFF, 0x00);
    }

    fn handleInterrupt(self: *CPU, interrupt: InterruptBit) void {
        self.ime_state = .disabled;
        self.halted = false;
        self.memory.acknowledgeInterrupt(interrupt);

        self.pushU16(self.pc);

        self.pc = switch (interrupt) {
            .vblank => 0x0040,
            .lcd => 0x0048,
            .timer => 0x0050,
            .serial => 0x0058,
            .joypad => 0x0060,
        };
    }


    // Individual instruction implementations
    fn nop(self: *CPU) void {
        _ = self;
    }

    fn ld_hl_ptr_plus_a(self: *CPU) void {
        const address = self.getU16Register(.hl);
        self.memory.write(address, self.a);
        self.setU16Register(.hl, self.getU16Register(.hl) + 1);
    }

    fn ld_a_hl_ptr_plus(self: *CPU) void {
        const address = self.getU16Register(.hl);
        self.a = self.memory.read(address, u8);
        self.setU16Register(.hl, self.getU16Register(.hl) + 1);
    }

    fn ld_hl_ptr_minus_a(self: *CPU) void {
        const address = self.getU16Register(.hl);
        self.memory.write(address, self.a);
        self.setU16Register(.hl, self.getU16Register(.hl) - 1);
    }

    fn ld_a_hl_ptr_minus(self: *CPU) void {
        const address = self.getU16Register(.hl);
        self.a = self.memory.read(address, u8);
        self.setU16Register(.hl, self.getU16Register(.hl) - 1);
    }

    fn rlca(self: *CPU) void {
        const bit7 = (self.a >> 7) & 1;
        self.a = (self.a << 1) | bit7;

        self.unsetFlag(.z);
        self.unsetFlag(.n);
        self.unsetFlag(.h);
        if (bit7 != 0) {
            self.setFlag(.c);
        } else {
            self.unsetFlag(.c);
        }
    }

    fn rla(self: *CPU) void {
        const carry_flag: u8 = (self.f >> 4) & 1;
        const bit7: u8 = self.a & 0b10000000;
        self.a <<= 1;
        self.a |= carry_flag;
        self.f = bit7 >> 3;
    }

    fn rra(self: *CPU) void {
        const carry_flag: u8 = (self.f << 3) & 0b10000000;
        const bit0: u8 = self.a & 1;
        self.a >>= 1;
        self.a |= carry_flag;
        self.f = bit0 << 4;
    }

    fn rrca(self: *CPU) void {
        const bit0: u8 = self.a & 1;
        self.a = (self.a >> 1) | (bit0 << 7);

        self.unsetFlag(.z);
        self.unsetFlag(.n);
        self.unsetFlag(.h);
        if (bit0 != 0) {
            self.setFlag(.c);
        } else {
            self.unsetFlag(.c);
        }
    }

    fn ld_a16_ptr_sp(self: *CPU) void {
        const address: u16 = self.memory.read(self.pc, u16);
        self.pc += 2;
        self.memory.write(address, self.sp);
    }

    fn ld_sp_d16(self: *CPU) void {
        const d16 = self.memory.read(self.pc, u16);
        self.pc += 2;
        self.sp = d16;
    }

    // TODO: Study complex stop flowchart
    fn stop(self: *CPU) void {

        const button_pressed= self.memory.isAnyButtonPressed();
        if (button_pressed) {
            if (self.memory.getPendingInterrupt()) |pending| {
                _ = pending;
                // STOP is a 1 byte opcode, mode doesn't change, DIV doesn't reset.
            } else {
                // STOP is a 2-byte opcode, HALT mode is entered, DIV is not reset
            }
        }

        
    }

    fn inc_hl_ptr(self: *CPU) void {
        const address = self.getU16Register(.hl);
        const old = self.memory.read(address, u8);
        const new = old +% 1;
        self.memory.write(address, new);
        self.updateIncFlags(old, new, false);
    }

    fn dec_hl_ptr(self: *CPU) void {
        const address = self.getU16Register(.hl);
        const old = self.memory.read(address, u8);
        const new = old -% 1;
        self.memory.write(address, new);
        self.updateIncFlags(old, new, true);
    }

    fn halt(self: *CPU) void {
        self.halted = true;
        const pending = self.memory.getPendingInterrupt();
        if (self.ime_state == .disabled and pending != null) {
            self.haltBug = true;
        }
    }

    fn ld_hl_ptr_d8(self: *CPU) void {
        const value = self.memory.read(self.pc, u8);
        self.pc += 1;
        self.memory.write(self.getU16Register(.hl), value);
    }

    fn daa(self: *CPU) void {
        const n_flag = self.readFlag(.n);
        const h_flag = self.readFlag(.h);
        const c_flag = self.readFlag(.c);

        var carry = false;
        var correction: u8 = 0;

        if (!n_flag) {
            if (c_flag or self.a > 0x99) {
                correction |= 0x60;
                carry = true;
            }
            if (h_flag or (self.a & 0x0F) > 0x09) {
                correction |= 0x06;
            }
            self.a +%= correction;
            if (c_flag) {
                correction += 0x60;
            }
            if (h_flag) {
                correction |= 0x06;
            }
            self.a -%= correction;
        }

        self.checkFlag(.z, self.a == 0);
        self.unsetFlag(.h);
        self.checkFlag(.c, carry);
    }

    fn scf(self: *CPU) void {
        self.setFlag(.c);
        self.unsetFlag(.n);
        self.unsetFlag(.h);
    }

    fn ccf(self: *CPU) void {
        const c_flag = self.readFlag(.c);
        self.checkFlag(.c, !c_flag);
        self.unsetFlag(.n);
        self.unsetFlag(.h);
    }

    fn ei(self: *CPU) void {
        self.ime_state = .enable_pending;
    }

    fn di(self: *CPU) void {
        self.ime_state = .disabled;
    }

    fn reti(self: *CPU) void {
        self.pc = self.popU16();
        self.ime_state = .enabled; // RETI enables immediately, no delay
    }

    fn unknown_opcode(self: *CPU) void {
        _ = self;
        @panic("Unknown opcode");
    }

    // Helper functions
    fn getRegister(self: *CPU, comptime reg: Register) *u8 {
        return switch (reg) {
            .a => &self.a,
            .f => &self.f,
            .b => &self.b,
            .c => &self.c,
            .d => &self.d,
            .e => &self.e,
            .h => &self.h,
            .l => &self.l,
        };
    }

    fn getU16Register(self: *CPU, comptime pair: U16Register) u16 {
        return switch (pair) {
            .bc => (@as(u16, self.b) << 8) | self.c,
            .de => (@as(u16, self.d) << 8) | self.e,
            .hl => (@as(u16, self.h) << 8) | self.l,
            .af => (@as(u16, self.a) << 8) | self.f,
            .sp => self.sp,
            .pc => self.pc,
        };
    }

    fn setU16Register(self: *CPU, comptime pair: U16Register, val: u16) void {
        switch (pair) {
            .bc => {
                self.b = @truncate(val >> 8);
                self.c = @truncate(val);
            },
            .de => {
                self.d = @truncate(val >> 8);
                self.e = @truncate(val);
            },
            .hl => {
                self.h = @truncate(val >> 8);
                self.l = @truncate(val);
            },
            .af => {
                self.a = @truncate(val >> 8);
                self.f = @truncate(val);
            },
            .sp => self.sp = val,
            .pc => self.pc = val,
        }
    }

    fn setFlag(self: *CPU, comptime flag: Flag) void {
        self.f |= (@as(u8, 1) << @intFromEnum(flag));
    }

    fn unsetFlag(self: *CPU, comptime flag: Flag) void {
        self.f &= ~(@as(u8, 1) << @intFromEnum(flag));
    }

    fn readFlag(self: *CPU, comptime flag: Flag) bool {
        return @as(u8, self.f >> @intFromEnum(flag)) == 1;
    }

    fn checkFlag(self: *CPU, comptime flag: Flag, condition: bool) void {
        if (condition) {
            self.setFlag(flag);
        } else {
            self.unsetFlag(flag);
        }
    }

    fn updateIncFlags(self: *CPU, old: u8, new: u8, comptime is_dec: bool) void {
        self.checkFlag(.z, new == 0);
        self.checkFlag(.n, is_dec);
        if (is_dec) {
            self.checkFlag(.h, (old & 0x0F) == 0);
        } else {
            self.checkFlag(.h, (old & 0x0F) + 1 > 0x0F);
        }
    }

    fn updateMathFlags(self: *CPU, comptime T: type, a: T, b: T, result: T, comptime is_subtract: bool) void {
        self.checkFlag(.z, result == 0);
        self.checkFlag(.n, is_subtract);

        if (T == u8) {
            if (is_subtract) {
                self.checkFlag(.h, (a & 0x0F) < (b & 0x0F));
                self.checkFlag(.c, a < b);
            } else {
                self.checkFlag(.h, (a & 0x0F) + (b & 0x0F) > 0x0F);
                self.checkFlag(.c, result < a);
            }
        } else if (T == u16) {
            if (is_subtract) {
                self.checkFlag(.h, (a & 0x0FFF) < (b & 0x0FFF));
                self.checkFlag(.c, a < b);
            } else {
                self.checkFlag(.h, (a & 0x0FFF) + (b & 0x0FFF) > 0x0FFF);
                self.checkFlag(.c, result < a);
            }
        } else {
            unreachable;
        }
    }

    fn pushU16(self: *CPU, value: u16) void {
        self.sp -%= 2;
        self.memory.write(self.sp, value);
    }

    fn popU16(self: *CPU) u16 {
        const value = self.memory.read(self.sp, u16);
        self.sp +%= 2;
        return value;
    }
};
