// Currently implementing every single instruction individually as a function
// Probably a better way to do it using comptime, not smart enough for that yet.
// Simply writing out a few functions for each instruction type and having claude
// copy paste the pattern and test after. Consider major refactor later for sake
// of conciseness. This works completely fine as well though.

const std = @import("std");

const InstructionFn = *const fn (*CPU) void;
const Flag = enum(u3) { z = 7, n = 6, h = 5, c = 4 };
const Register = enum(u3) { a = 0, f = 1, b = 2, c = 3, d = 4, e = 5, h = 6, l = 7 };

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
            cpu.getRegister(dst).* = cpu.memory[cpu.getHL()];
        }
    }.f;
}

fn makeLdHlPtrReg(comptime src: Register) InstructionFn {
    return struct {
        fn f(cpu: *CPU) void {
            cpu.memory[cpu.getHL()] = cpu.getRegister(src).*;
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

// Instruction table matching opcodes to their corresponding functions
const instruction_table = blk: {
    var table = [_]InstructionFn{CPU.unknown_opcode} ** 256;
    table[0x00] = CPU.nop;
    table[0x01] = CPU.ld_bc_d16;
    table[0x02] = CPU.ld_bc_ptr_a;
    table[0x03] = CPU.inc_bc;
    table[0x04] = makeIncReg(.b);
    table[0x05] = makeDecReg(.b);
    table[0x06] = CPU.ld_b_d8;
    table[0x07] = CPU.rlca;
    table[0x08] = CPU.ld_a16_ptr_sp;
    table[0x09] = CPU.add_hl_bc;
    table[0x0A] = CPU.ld_a_bc_ptr;
    table[0x0B] = CPU.dec_bc;
    table[0x0C] = makeIncReg(.c);
    table[0x0D] = makeDecReg(.c);
    table[0x0E] = CPU.ld_c_d8;
    table[0x0F] = CPU.rrca;
    table[0x11] = CPU.ld_de_d16;
    table[0x12] = CPU.ld_de_ptr_a;
    table[0x13] = CPU.inc_de;
    table[0x14] = makeIncReg(.d);
    table[0x15] = makeDecReg(.d);
    table[0x16] = CPU.ld_d_d8;
    table[0x17] = CPU.rla;
    table[0x19] = CPU.add_hl_de;
    table[0x1A] = CPU.ld_a_de_ptr;
    table[0x1B] = CPU.dec_de;
    table[0x1C] = makeIncReg(.e);
    table[0x1D] = makeDecReg(.e);
    table[0x1E] = CPU.ld_e_d8;
    table[0x1F] = CPU.rra;
    table[0x18] = CPU.jr_r8;
    table[0x20] = CPU.jr_nz_r8;
    table[0x21] = CPU.ld_hl_d16;
    table[0x22] = CPU.ld_hl_ptr_plus_a;
    table[0x23] = CPU.inc_hl;
    table[0x24] = makeIncReg(.h);
    table[0x25] = makeDecReg(.h);
    table[0x26] = CPU.ld_h_d8;
    table[0x28] = CPU.jr_z_r8;
    table[0x29] = CPU.add_hl_hl;
    table[0x2A] = CPU.ld_a_hl_ptr_plus;
    table[0x2B] = CPU.dec_hl;
    table[0x2C] = makeIncReg(.l);
    table[0x2D] = makeDecReg(.l);
    table[0x2E] = CPU.ld_l_d8;
    table[0x30] = CPU.jr_nc_r8;
    table[0x32] = CPU.ld_hl_ptr_minus_a;
    table[0x33] = CPU.inc_sp;
    table[0x34] = CPU.inc_hl_ptr;
    table[0x35] = CPU.dec_hl_ptr;
    table[0x36] = CPU.ld_hl_ptr_d8;
    table[0x38] = CPU.jr_c_r8;
    table[0x39] = CPU.add_hl_sp;
    table[0x3A] = CPU.ld_a_hl_ptr_minus;
    table[0x3B] = CPU.dec_sp;
    table[0x3C] = makeIncReg(.a);
    table[0x3D] = makeDecReg(.a);
    table[0x3E] = CPU.ld_a_d8;
    table[0x40] = CPU.nop; // LD B,B (NOP)
    table[0x41] = makeLdRegReg(.b, .c);
    table[0x42] = makeLdRegReg(.b, .d);
    table[0x43] = makeLdRegReg(.b, .e);
    table[0x44] = makeLdRegReg(.b, .h);
    table[0x45] = makeLdRegReg(.b, .l);
    table[0x46] = makeLdRegHlPtr(.b);
    table[0x47] = makeLdRegReg(.b, .a);
    table[0x48] = makeLdRegReg(.c, .b);
    table[0x49] = CPU.nop; // LD C,C (NOP)
    table[0x4A] = makeLdRegReg(.c, .d);
    table[0x4B] = makeLdRegReg(.c, .e);
    table[0x4C] = makeLdRegReg(.c, .h);
    table[0x4D] = makeLdRegReg(.c, .l);
    table[0x4E] = makeLdRegHlPtr(.c);
    table[0x4F] = makeLdRegReg(.c, .a);
    table[0x50] = makeLdRegReg(.d, .b);
    table[0x51] = makeLdRegReg(.d, .c);
    table[0x52] = CPU.nop; // LD D,D (NOP)
    table[0x53] = makeLdRegReg(.d, .e);
    table[0x54] = makeLdRegReg(.d, .h);
    table[0x55] = makeLdRegReg(.d, .l);
    table[0x56] = makeLdRegHlPtr(.d);
    table[0x57] = makeLdRegReg(.d, .a);
    table[0x58] = makeLdRegReg(.e, .b);
    table[0x59] = makeLdRegReg(.e, .c);
    table[0x5A] = makeLdRegReg(.e, .d);
    table[0x5B] = CPU.nop; // LD E,E (NOP)
    table[0x5C] = makeLdRegReg(.e, .h);
    table[0x5D] = makeLdRegReg(.e, .l);
    table[0x5E] = makeLdRegHlPtr(.e);
    table[0x5F] = makeLdRegReg(.e, .a);
    table[0x60] = makeLdRegReg(.h, .b);
    table[0x61] = makeLdRegReg(.h, .c);
    table[0x62] = makeLdRegReg(.h, .d);
    table[0x63] = makeLdRegReg(.h, .e);
    table[0x64] = CPU.nop; // LD H,H (NOP)
    table[0x65] = makeLdRegReg(.h, .l);
    table[0x66] = makeLdRegHlPtr(.h);
    table[0x67] = makeLdRegReg(.h, .a);
    table[0x68] = makeLdRegReg(.l, .b);
    table[0x69] = makeLdRegReg(.l, .c);
    table[0x6A] = makeLdRegReg(.l, .d);
    table[0x6B] = makeLdRegReg(.l, .e);
    table[0x6C] = makeLdRegReg(.l, .h);
    table[0x6D] = CPU.nop; // LD L,L (NOP)
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
    table[0x7F] = CPU.nop; // LD A,A (NOP)
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
    table[0xC6] = CPU.add_a_d8;
    table[0xD6] = CPU.sub_a_d8;
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

    stopped: bool = false,
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

    fn ld_bc_ptr_a(self: *CPU) void {
        const address = self.getBC();
        self.memory[address] = self.a;
    }

    fn ld_b_d8(self: *CPU) void {
        self.b = self.memory[self.pc];
        self.pc += 1;
    }

    fn ld_a_bc_ptr(self: *CPU) void {
        const address = self.getBC();
        self.a = self.memory[address];
    }

    fn ld_c_d8(self: *CPU) void {
        self.c = self.memory[self.pc];
        self.pc += 1;
    }

    fn ld_de_ptr_a(self: *CPU) void {
        const address = self.getDE();
        self.memory[address] = self.a;
    }

    fn ld_d_d8(self: *CPU) void {
        self.d = self.memory[self.pc];
        self.pc += 1;
    }

    fn ld_a_de_ptr(self: *CPU) void {
        const address = self.getDE();
        self.a = self.memory[address];
    }

    fn ld_e_d8(self: *CPU) void {
        self.e = self.memory[self.pc];
        self.pc += 1;
    }

    fn ld_hl_ptr_plus_a(self: *CPU) void {
        const address = self.getHL();
        self.memory[address] = self.a;
        self.setHL(self.getHL() + 1);
    }

    fn ld_h_d8(self: *CPU) void {
        self.h = self.memory[self.pc];
        self.pc += 1;
    }

    fn ld_a_hl_ptr_plus(self: *CPU) void {
        const address = self.getHL();
        self.a = self.memory[address];
        self.setHL(self.getHL() + 1);
    }

    fn ld_l_d8(self: *CPU) void {
        self.l = self.memory[self.pc];
        self.pc += 1;
    }

    fn ld_hl_ptr_minus_a(self: *CPU) void {
        const address = self.getHL();
        self.memory[address] = self.a;
        self.setHL(self.getHL() - 1);
    }

    fn ld_a_hl_ptr_minus(self: *CPU) void {
        const address = self.getHL();
        self.a = self.memory[address];
        self.setHL(self.getHL() - 1);
    }

    fn inc_bc(self: *CPU) void {
        self.setBC(self.getBC() + 1);
    }


    fn add_hl_bc(self: *CPU) void {
        const hl = self.getHL();
        const bc = self.getBC();
        const result = hl +% bc;
        self.updateFlags(u16, hl, bc, result, false);
        self.setHL(result);
    }

    fn dec_bc(self: *CPU) void {
        self.setBC(self.getBC() -% 1);
    }


    fn add_hl_de(self: *CPU) void {
        const hl = self.getHL();
        const de = self.getDE();
        const result = hl +% de;
        self.updateFlags(u16, hl, de, result, false);
        self.setHL(result);
    }

    fn dec_de(self: *CPU) void {
        const de = self.getDE();
        self.setDE(de -% 1);
        self.updateFlags(u16, de, 1, self.getDE(), true);
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

    fn jr_r8(self: *CPU) void {
        const offset: i8 = @bitCast(self.memory[self.pc]);
        self.pc += 1;
        const offset_i16: i16 = offset;
        const offset_u16: u16 = @bitCast(offset_i16);
        self.pc +%= offset_u16;
    }

    fn jr_nz_r8(self: *CPU) void {
        const offset: i8 = @bitCast(self.memory[self.pc]);
        self.pc += 1;
        if ((self.f & 0b10000000) != 0b10000000) {
            const offset_i16: i16 = offset;
            const offset_u16: u16 = @bitCast(offset_i16);
            self.pc +%= offset_u16;
        }
    }

    fn jr_z_r8(self: *CPU) void {
        const offset: i8 = @bitCast(self.memory[self.pc]);
        self.pc += 1;
        if ((self.f & 0b10000000) == 0b10000000) {
            const offset_i16: i16 = offset;
            const offset_u16: u16 = @bitCast(offset_i16);
            self.pc +%= offset_u16;
        }
    }

    fn jr_nc_r8(self: *CPU) void {
        const offset: i8 = @bitCast(self.memory[self.pc]);
        self.pc += 1;
        if ((self.f & 0b00010000) != 0b00010000) {
            const offset_i16: i16 = offset;
            const offset_u16: u16 = @bitCast(offset_i16);
            self.pc +%= offset_u16;
        }
    }

    fn jr_c_r8(self: *CPU) void {
        const offset: i8 = @bitCast(self.memory[self.pc]);
        self.pc += 1;
        if ((self.f & 0b00010000) == 0b00010000) {
            const offset_i16: i16 = offset;
            const offset_u16: u16 = @bitCast(offset_i16);
            self.pc +%= offset_u16;
        }
    }

    fn ld_bc_d16(self: *CPU) void {
        const d16 = std.mem.readInt(u16, self.memory[self.pc..][0..2], .little);
        self.pc += 2;
        self.setBC(d16);
    }

    fn ld_de_d16(self: *CPU) void {
        const d16 = std.mem.readInt(u16, self.memory[self.pc..][0..2], .little);
        self.pc += 2;
        self.setDE(d16);
    }

    fn ld_hl_d16(self: *CPU) void {
        const d16 = std.mem.readInt(u16, self.memory[self.pc..][0..2], .little);
        self.pc += 2;
        self.setHL(d16);
    }

    fn ld_sp_d16(self: *CPU) void {
        const d16 = std.mem.readInt(u16, self.memory[self.pc..][0..2], .little);
        self.pc += 2;
        self.sp = d16;
    }

    fn add_hl_hl(self: *CPU) void {
        const hl = self.getHL();
        const result = hl +% hl;
        self.setHL(result);
        self.updateFlags(u16, hl, hl, result, false);
    }

    fn add_hl_sp(self: *CPU) void {
        const hl = self.getHL();
        const result = hl + self.sp;
        self.setHL(result);
        self.updateFlags(u16, hl, self.sp, result, false);
    }

    fn dec_HL(self: *CPU) void {
        const old = self.getHL();
        self.setHL(old -% 1);
        self.updateFlags(u16, old, 1, true);
    }

    // TODO: Need interrupts and halt states to implement
    fn stop(self: *CPU) void {
        self.stopped = true;
    }

    fn inc_hl(self: *CPU) void {
        const old = self.getHL();
        self.setHL(old +% 1);
    }

    fn inc_hl_ptr(self: *CPU) void {
        const address = self.getHL();
        const old = self.memory[address];
        self.memory[address] +%= 1;
        self.updateFlags(u8, old, 1, self.memory[address], false);
    }

    fn inc_de(self: *CPU) void {
        self.setDE(self.getDE() +% 1);
    }

    fn inc_sp(self: *CPU) void {
        self.sp +%= 1;
    }

    fn dec_hl(self: *CPU) void {
        self.setHL(self.getHL() -% 1);
    }

    fn dec_hl_ptr(self: *CPU) void {
        const address = self.getHL();
        const old = self.memory[address];
        self.memory[address] -%= 1;
        self.updateFlags(u8, old, 1, self.memory[address], true);
    }

    fn dec_sp(self: *CPU) void {
        self.sp -%= 1;
    }

    fn add_a_hl_ptr(self: *CPU) void {
        const address = self.getHL();
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
        const address = self.getHL();
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

    fn ld_a_d8(self: *CPU) void {
        self.a = self.memory[self.pc];
        self.pc += 1;
    }

    fn ld_hl_ptr_d8(self: *CPU) void {
        const value = self.memory[self.pc];
        self.pc += 1;
        self.memory[self.getHL()] = value;
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
        self.a &= self.memory[self.getHL()];
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
        self.a |= self.memory[self.getHL()];
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
        self.a ^= self.memory[self.getHL()];
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
        const value = self.memory[self.getHL()];
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

    // Helper functions
    fn getBC(self: *CPU) u16 {
        return (@as(u16, self.b) << 8) | self.c;
    }

    fn getDE(self: *CPU) u16 {
        return (@as(u16, self.d) << 8) | self.e;
    }

    fn getHL(self: *CPU) u16 {
        return (@as(u16, self.h) << 8) | self.l;
    }

    fn setBC(self: *CPU, val: u16) void {
        self.b = @truncate(val >> 8);
        self.c = @truncate(val);
    }

    fn setDE(self: *CPU, val: u16) void {
        self.d = @truncate(val >> 8);
        self.e = @truncate(val);
    }

    fn setHL(self: *CPU, val: u16) void {
        self.h = @truncate(val >> 8);
        self.l = @truncate(val);
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