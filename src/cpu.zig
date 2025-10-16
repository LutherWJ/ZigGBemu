// Currently implementing every single instruction individually as a function
// Probably a better way to do it using comptime, not smart enough for that yet.
// Simply writing out a few functions for each instruction type and having claude
// copy paste the pattern and test after. Consider major refactor later for sake
// of conciseness. This works completely fine as well though.

const std = @import("std");
const Memory = @import("memory.zig").Memory;

const InstructionFn = *const fn (*CPU) void;
const Flag = enum(u3) { z = 7, n = 6, h = 5, c = 4 };
const Register = enum { a, f, b, c, d, e, h, l };
const U16Register = enum { bc, de, hl, af, sp, pc };
const Conditions = enum { always, nz, z, nc, c }; // Used for conditional instructions like CALL or JP

// Comptime function generators for reducing boilerplate
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
            cpu.getRegister(dst).* = cpu.memory[cpu.getU16Register(.hl)];
        }
    }.f;
}

fn makeLdRegU16Ptr(comptime dst: Register, comptime src: U16Register) InstructionFn {
    return struct {
        fn f(cpu: *CPU) void {
            const address = cpu.getU16Register(src);
            cpu.getRegister(dst).* = cpu.memory[address];
        }
    }.f;
}

fn makeLdU16PtrReg(comptime dst: U16Register, comptime src: Register) InstructionFn {
    return struct {
        fn f(cpu: *CPU) void {
            const address = cpu.getU16Register(dst);
            cpu.memory[address] = cpu.getRegister(src).*;
        }
    }.f;
}

fn makeLdRegD8(comptime dst: Register) InstructionFn {
    return struct {
        fn f(cpu: *CPU) void {
            cpu.getRegister(dst).* = cpu.memory[cpu.pc];
            cpu.pc += 1;
        }
    }.f;
}

fn makeLdU16RegD16(comptime dst: U16Register) InstructionFn {
    return struct {
        fn f(cpu: *CPU) void {
            const d16 = std.mem.readInt(u16, cpu.memory[cpu.pc..][0..2], .little);
            cpu.pc += 2;
            cpu.setU16Register(dst, d16);
        }
    }.f;
}

fn makeLdHlPtrReg(comptime src: Register) InstructionFn {
    return struct {
        fn f(cpu: *CPU) void {
            cpu.memory[cpu.getU16Register(.hl)] = cpu.getRegister(src).*;
        }
    }.f;
}

fn makeAddReg(comptime src: Register) InstructionFn {
    return struct {
        fn f(cpu: *CPU) void {
            const old = cpu.a;
            const value = cpu.getRegister(src).*;
            cpu.a +%= value;
            cpu.updateFlags(u8, old, value, cpu.a, false);
        }
    }.f;
}

fn makeSubReg(comptime src: Register) InstructionFn {
    return struct {
        fn f(cpu: *CPU) void {
            const old = cpu.a;
            const value = cpu.getRegister(src).*;
            cpu.a -%= value;
            cpu.updateFlags(u8, old, value, cpu.a, true);
        }
    }.f;
}

fn makeAndReg(comptime src: Register) InstructionFn {
    return struct {
        fn f(cpu: *CPU) void {
            const value = cpu.getRegister(src).*;
            cpu.a &= value;
            cpu.checkFlag(.z, cpu.a == 0);
            cpu.unsetFlag(.n);
            cpu.setFlag(.h);
            cpu.unsetFlag(.c);
        }
    }.f;
}

fn makeOrReg(comptime src: Register) InstructionFn {
    return struct {
        fn f(cpu: *CPU) void {
            const value = cpu.getRegister(src).*;
            cpu.a |= value;
            cpu.checkFlag(.z, cpu.a == 0);
            cpu.unsetFlag(.n);
            cpu.unsetFlag(.h);
            cpu.unsetFlag(.c);
        }
    }.f;
}

fn makeXorReg(comptime src: Register) InstructionFn {
    return struct {
        fn f(cpu: *CPU) void {
            const value = cpu.getRegister(src).*;
            cpu.a ^= value;
            cpu.checkFlag(.z, cpu.a == 0);
            cpu.unsetFlag(.n);
            cpu.unsetFlag(.h);
            cpu.unsetFlag(.c);
        }
    }.f;
}

fn makeCpReg(comptime src: Register) InstructionFn {
    return struct {
        fn f(cpu: *CPU) void {
            const value = cpu.getRegister(src).*;
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
            cpu.updateFlags(u8, old, 1, reg.*, false);
        }
    }.f;
}

fn makeDecReg(comptime dst: Register) InstructionFn {
    return struct {
        fn f(cpu: *CPU) void {
            const reg = cpu.getRegister(dst);
            const old = reg.*;
            reg.* -%= 1;
            cpu.updateFlags(u8, old, 1, reg.*, true);
        }
    }.f;
}

// Simple 16-bit inc/dec (don't affect flags)
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

// ADD HL generator
fn makeAddHl(comptime src: U16Register) InstructionFn {
    return struct {
        fn f(cpu: *CPU) void {
            const hl = cpu.getU16Register(.hl);
            const value = cpu.getU16Register(src);
            const result = hl +% value;
            cpu.updateFlags(u16, hl, value, result, false);
            cpu.setU16Register(.hl, result);
        }
    }.f;
}

fn makeJumpRelative(comptime cond: Conditions) InstructionFn {
    return struct {
        fn f(cpu: *CPU) void {
            const offset: i8 = @bitCast(cpu.memory[cpu.pc]);
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
            const address: u16 = std.mem.readInt(u16, cpu.memory[cpu.pc..][0..2], .little);
            cpu.pc +%= 2;

            const should_call = switch (cond) {
                .always => true,
                .nz => (cpu.f & 0b10000000) != 0b10000000,
                .z => (cpu.f & 0b10000000) == 0b10000000,
                .nc => (cpu.f & 0b00010000) != 0b00010000,
                .c => (cpu.f & 0b00010000) == 0b00010000,
            };

            if (should_call) {
                cpu.sp -%= 2;
                const value = std.mem.readInt(u16, cpu.memory[cpu.pc..][0..2], .little);
                std.mem.writeInt(u16, cpu.memory[cpu.sp..][0..2], value, .little);
                cpu.pc = address;
            }
        }
    }.f;
}

fn makePush(comptime pair: U16Register) InstructionFn {
    return struct {
        fn f(cpu: *CPU) void {
            const value = cpu.getU16Register(pair);
            cpu.sp -%= 1;
            cpu.memory[cpu.sp] = @truncate(value >> 8); // High byte
            cpu.sp -%= 1;
            cpu.memory[cpu.sp] = @truncate(value); // Low byte
        }
    }.f;
}

fn makePop(comptime pair: U16Register) InstructionFn {
    return struct {
        fn f(cpu: *CPU) void {
            const low = cpu.memory[cpu.sp];
            cpu.sp +%= 1;
            const high = cpu.memory[cpu.sp];
            cpu.sp +%= 1;
            const value = (@as(u16, high) << 8) | low;
            cpu.setU16Register(pair, value);
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
    table[0x33] = CPU.inc_sp;
    table[0x34] = CPU.inc_hl_ptr;
    table[0x35] = CPU.dec_hl_ptr;
    table[0x36] = CPU.ld_hl_ptr_d8;
    table[0x38] = makeJumpRelative(.c);
    table[0x39] = makeAddHl(.sp);
    table[0x3A] = CPU.ld_a_hl_ptr_minus;
    table[0x3B] = CPU.dec_sp;
    table[0x3C] = makeIncReg(.a);
    table[0x3D] = makeDecReg(.a);
    table[0x3E] = makeLdRegD8(.a);
    table[0x40] = CPU.nop; // LD B,B (NOP)
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
    table[0x80] = makeAddReg(.b);
    table[0x81] = makeAddReg(.c);
    table[0x82] = makeAddReg(.d);
    table[0x83] = makeAddReg(.e);
    table[0x84] = makeAddReg(.h);
    table[0x85] = makeAddReg(.l);
    table[0x86] = CPU.add_a_hl_ptr;
    table[0x87] = makeAddReg(.a);
    table[0x90] = makeSubReg(.b);
    table[0x91] = makeSubReg(.c);
    table[0x92] = makeSubReg(.d);
    table[0x93] = makeSubReg(.e);
    table[0x94] = makeSubReg(.h);
    table[0x95] = makeSubReg(.l);
    table[0x96] = CPU.sub_a_hl_ptr;
    table[0x97] = makeSubReg(.a);
    table[0xB8] = makeCpReg(.b);
    table[0xB9] = makeCpReg(.c);
    table[0xBA] = makeCpReg(.d);
    table[0xBB] = makeCpReg(.e);
    table[0xBC] = makeCpReg(.h);
    table[0xBD] = makeCpReg(.l);
    table[0xBE] = CPU.cp_hl_ptr;
    table[0xBF] = makeCpReg(.a);
    table[0xC1] = makePop(.bc);
    table[0xC5] = makePush(.bc);
    table[0xC4] = makeCall(.nz);
    table[0xC6] = CPU.add_a_d8;
    table[0xCC] = makeCall(.z);
    table[0xD1] = makePop(.de);
    table[0xD4] = makeCall(.nc);
    table[0xD5] = makePush(.de);
    table[0xD6] = CPU.sub_a_d8;
    table[0xDC] = makeCall(.c);
    table[0xDD] = makeCall(.always);
    table[0xE1] = makePop(.hl);
    table[0xE5] = makePush(.hl);
    table[0xF1] = makePop(.af);
    table[0xF5] = makePush(.af);
    table[0xFE] = CPU.cp_d8;
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

    stopped: u1 = 0,
    ime_flag: u1 = 0, // TODO: implement halt hardware bug later down the line
    memory: [0x10000]u8 = undefined,

    pub fn step(self: *CPU) !void {
        const opcode = self.memory[self.pc];
        self.pc += 1;
        instruction_table[opcode](self);
    }

    // Individual instruction implementations
    fn nop(self: *CPU) void {
        _ = self;
    }

    fn ld_hl_ptr_plus_a(self: *CPU) void {
        const address = self.getU16Register(.hl);
        self.memory[address] = self.a;
        self.setU16Register(.hl, self.getU16Register(.hl) + 1);
    }

    fn ld_a_hl_ptr_plus(self: *CPU) void {
        const address = self.getU16Register(.hl);
        self.a = self.memory[address];
        self.setU16Register(.hl, self.getU16Register(.hl) + 1);
    }

    fn ld_hl_ptr_minus_a(self: *CPU) void {
        const address = self.getU16Register(.hl);
        self.memory[address] = self.a;
        self.setU16Register(.hl, self.getU16Register(.hl) - 1);
    }

    fn ld_a_hl_ptr_minus(self: *CPU) void {
        const address = self.getU16Register(.hl);
        self.a = self.memory[address];
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
        const address: u16 = std.mem.readInt(u16, self.memory[self.pc..][0..2], .little);
        self.pc += 2;
        std.mem.writeInt(u16, self.memory[address..][0..2], self.sp, .little);
    }

    fn ld_sp_d16(self: *CPU) void {
        const d16 = std.mem.readInt(u16, self.memory[self.pc..][0..2], .little);
        self.pc += 2;
        self.sp = d16;
    }

    fn dec_HL(self: *CPU) void {
        const old = self.getU16Register(.hl);
        self.setU16Register(.hl, old -% 1);
        self.updateFlags(u16, old, 1, true);
    }

    // TODO: Need interrupts and halt states to implement
    fn stop(self: *CPU) void {
        self.stopped = true;
    }

    fn inc_hl_ptr(self: *CPU) void {
        const address = self.getU16Register(.hl);
        const old = self.memory[address];
        self.memory[address] +%= 1;
        self.updateFlags(u8, old, 1, self.memory[address], false);
    }

    fn inc_sp(self: *CPU) void {
        self.sp +%= 1;
    }

    fn dec_hl_ptr(self: *CPU) void {
        const address = self.getU16Register(.hl);
        const old = self.memory[address];
        self.memory[address] -%= 1;
        self.updateFlags(u8, old, 1, self.memory[address], true);
    }

    fn dec_sp(self: *CPU) void {
        self.sp -%= 1;
    }

    fn add_a_hl_ptr(self: *CPU) void {
        const address = self.getU16Register(.hl);
        const old = self.a;
        const value = self.memory[address];
        self.a +%= value;
        self.updateFlags(u8, old, value, self.a, false);
    }

    fn add_a_d8(self: *CPU) void {
        const old = self.a;
        const value = self.memory[self.pc];
        self.pc += 1;
        self.a +%= value;
        self.updateFlags(u8, old, value, self.a, false);
    }

    fn sub_a_hl_ptr(self: *CPU) void {
        const address = self.getU16Register(.hl);
        const old = self.a;
        const value = self.memory[address];
        self.a -%= value;
        self.updateFlags(u8, old, value, self.a, true);
    }

    fn sub_a_d8(self: *CPU) void {
        const old = self.a;
        const value = self.memory[self.pc];
        self.pc += 1;
        self.a -%= value;
        self.updateFlags(u8, old, value, self.a, true);
    }

    fn halt(self: *CPU) void {
        _ = self;
        // TODO: implement halt
    }

    fn ld_hl_ptr_d8(self: *CPU) void {
        const value = self.memory[self.pc];
        self.pc += 1;
        self.memory[self.getU16Register(.hl)] = value;
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

    fn and_hl_ptr(self: *CPU) void {
        self.a &= self.memory[self.getU16Register(.hl)];
        self.checkFlag(.z, self.a == 0);
        self.unsetFlag(.n);
        self.setFlag(.h);
        self.unsetFlag(.c);
    }

    fn and_d8(self: *CPU) void {
        const value = self.memory[self.pc];
        self.pc += 1;
        self.a &= value;
        self.checkFlag(.z, self.a == 0);
        self.unsetFlag(.n);
        self.setFlag(.h);
        self.unsetFlag(.c);
    }

    fn or_hl_ptr(self: *CPU) void {
        self.a |= self.memory[self.getU16Register(.hl)];
        self.checkFlag(.z, self.a == 0);
        self.unsetFlag(.n);
        self.unsetFlag(.h);
        self.unsetFlag(.c);
    }

    fn or_d8(self: *CPU) void {
        const value = self.memory[self.pc];
        self.pc += 1;
        self.a |= value;
        self.checkFlag(.z, self.a == 0);
        self.unsetFlag(.n);
        self.unsetFlag(.h);
        self.unsetFlag(.c);
    }

    fn xor_hl_ptr(self: *CPU) void {
        self.a ^= self.memory[self.getU16Register(.hl)];
        self.checkFlag(.z, self.a == 0);
        self.unsetFlag(.n);
        self.unsetFlag(.h);
        self.unsetFlag(.c);
    }

    fn xor_d8(self: *CPU) void {
        const value = self.memory[self.pc];
        self.pc += 1;
        self.a ^= value;
        self.checkFlag(.z, self.a == 0);
        self.unsetFlag(.n);
        self.unsetFlag(.h);
        self.unsetFlag(.c);
    }

    fn adc_a_b(self: *CPU) void {
        const old = self.a;
        const carry: u8 = if (self.readFlag(.c)) 1 else 0;
        self.a +%= self.b +% carry;
        self.updateFlags(u8, old, self.b + carry, self.a, false);
    }

    fn sbc_a_b(self: *CPU) void {
        const old = self.a;
        const carry: u8 = if (self.readFlag(.c)) 1 else 0;
        self.a -%= self.b +% carry;
        self.updateFlags(u8, old, self.b + carry, self.a, true);
    }

    fn cp_hl_ptr(self: *CPU) void {
        const value = self.memory[self.getU16Register(.hl)];
        const result = self.a -% value;
        self.checkFlag(.z, result == 0);
        self.setFlag(.n);
        self.checkFlag(.h, (self.a & 0x0F) < (value & 0x0F));
        self.checkFlag(.c, self.a < value);
    }

    fn cp_d8(self: *CPU) void {
        const value = self.memory[self.pc];
        self.pc += 1;
        const result = self.a -% value;
        self.checkFlag(.z, result == 0);
        self.setFlag(.n);
        self.checkFlag(.h, (self.a & 0x0F) < (value & 0x0F));
        self.checkFlag(.c, self.a < value);
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

    // Helper function to update flags after arithmetic.
    fn updateFlags(self: *CPU, comptime T: type, a: T, b: T, result: T, comptime is_subtract: bool) void {
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
};
