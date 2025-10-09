// Currently implementing every single instruction individually as a function
// Probably a better way to do it using comptime, not smart enough for that yet.
// Simply writing out a few functions for each instruction type and having claude
// copy paste the pattern and test after. Consider major refactor later for sake
// of conciseness. This works completely fine as well though.

const std = @import("std");

const InstructionFn = *const fn (*CPU) void;
const Flag = enum(u3) { z = 7, n = 6, h = 5, c = 4 };

// Instruction table matching opcodes to their corresponding functions
const instruction_table = blk: {
    var table = [_]InstructionFn{CPU.unknown_opcode} ** 256;
    table[0x00] = CPU.nop;
    table[0x01] = CPU.ld_bc_d16;
    table[0x02] = CPU.ld_bc_ptr_a;
    table[0x03] = CPU.inc_bc;
    table[0x04] = CPU.inc_b;
    table[0x05] = CPU.dec_b;
    table[0x06] = CPU.ld_b_d8;
    table[0x07] = CPU.rlca;
    table[0x08] = CPU.ld_a16_ptr_sp;
    table[0x09] = CPU.add_hl_bc;
    table[0x0A] = CPU.ld_a_bc_ptr;
    table[0x0B] = CPU.dec_bc;
    table[0x0C] = CPU.inc_c;
    table[0x0D] = CPU.dec_c;
    table[0x0E] = CPU.ld_c_d8;
    table[0x0F] = CPU.rrca;
    table[0x11] = CPU.ld_de_d16;
    table[0x12] = CPU.ld_de_ptr_a;
    table[0x13] = CPU.inc_de;
    table[0x14] = CPU.inc_d;
    table[0x15] = CPU.dec_d;
    table[0x16] = CPU.ld_d_d8;
    table[0x17] = CPU.rla;
    table[0x19] = CPU.add_hl_de;
    table[0x1A] = CPU.ld_a_de_ptr;
    table[0x1B] = CPU.dec_de;
    table[0x1C] = CPU.inc_e;
    table[0x1D] = CPU.dec_e;
    table[0x1E] = CPU.ld_e_d8;
    table[0x1F] = CPU.rra;
    table[0x18] = CPU.jr_r8;
    table[0x20] = CPU.jr_nz_r8;
    table[0x21] = CPU.ld_hl_d16;
    table[0x22] = CPU.ld_hl_ptr_plus_a;
    table[0x23] = CPU.inc_hl;
    table[0x24] = CPU.inc_h;
    table[0x25] = CPU.dec_h;
    table[0x26] = CPU.ld_h_d8;
    table[0x28] = CPU.jr_z_r8;
    table[0x29] = CPU.add_hl_hl;
    table[0x2A] = CPU.ld_a_hl_ptr_plus;
    table[0x2B] = CPU.dec_hl;
    table[0x2C] = CPU.inc_l;
    table[0x2D] = CPU.dec_l;
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
    table[0x3C] = CPU.inc_a;
    table[0x3D] = CPU.dec_a;
    table[0x3E] = CPU.ld_a_d8;
    table[0x40] = CPU.ld_b_b;
    table[0x41] = CPU.ld_b_c;
    table[0x42] = CPU.ld_b_d;
    table[0x43] = CPU.ld_b_e;
    table[0x44] = CPU.ld_b_h;
    table[0x45] = CPU.ld_b_l;
    table[0x46] = CPU.ld_b_hl_ptr;
    table[0x47] = CPU.ld_b_a;
    table[0x48] = CPU.ld_c_b;
    table[0x49] = CPU.ld_c_c;
    table[0x4A] = CPU.ld_c_d;
    table[0x4B] = CPU.ld_c_e;
    table[0x4C] = CPU.ld_c_h;
    table[0x4D] = CPU.ld_c_l;
    table[0x4E] = CPU.ld_c_hl_ptr;
    table[0x4F] = CPU.ld_c_a;
    table[0x50] = CPU.ld_d_b;
    table[0x51] = CPU.ld_d_c;
    table[0x52] = CPU.ld_d_d;
    table[0x53] = CPU.ld_d_e;
    table[0x54] = CPU.ld_d_h;
    table[0x55] = CPU.ld_d_l;
    table[0x56] = CPU.ld_d_hl_ptr;
    table[0x57] = CPU.ld_d_a;
    table[0x58] = CPU.ld_e_b;
    table[0x59] = CPU.ld_e_c;
    table[0x5A] = CPU.ld_e_d;
    table[0x5B] = CPU.ld_e_e;
    table[0x5C] = CPU.ld_e_h;
    table[0x5D] = CPU.ld_e_l;
    table[0x5E] = CPU.ld_e_hl_ptr;
    table[0x5F] = CPU.ld_e_a;
    table[0x60] = CPU.ld_h_b;
    table[0x61] = CPU.ld_h_c;
    table[0x62] = CPU.ld_h_d;
    table[0x63] = CPU.ld_h_e;
    table[0x64] = CPU.ld_h_h;
    table[0x65] = CPU.ld_h_l;
    table[0x66] = CPU.ld_h_hl_ptr;
    table[0x67] = CPU.ld_h_a;
    table[0x68] = CPU.ld_l_b;
    table[0x69] = CPU.ld_l_c;
    table[0x6A] = CPU.ld_l_d;
    table[0x6B] = CPU.ld_l_e;
    table[0x6C] = CPU.ld_l_h;
    table[0x6D] = CPU.ld_l_l;
    table[0x6E] = CPU.ld_l_hl_ptr;
    table[0x6F] = CPU.ld_l_a;
    table[0x70] = CPU.ld_hl_ptr_b;
    table[0x71] = CPU.ld_hl_ptr_c;
    table[0x72] = CPU.ld_hl_ptr_d;
    table[0x73] = CPU.ld_hl_ptr_e;
    table[0x74] = CPU.ld_hl_ptr_h;
    table[0x75] = CPU.ld_hl_ptr_l;
    table[0x76] = CPU.halt;
    table[0x77] = CPU.ld_hl_ptr_a;
    table[0x78] = CPU.ld_a_b;
    table[0x79] = CPU.ld_a_c;
    table[0x7A] = CPU.ld_a_d;
    table[0x7B] = CPU.ld_a_e;
    table[0x7C] = CPU.ld_a_h;
    table[0x7D] = CPU.ld_a_l;
    table[0x7E] = CPU.ld_a_hl_ptr;
    table[0x7F] = CPU.ld_a_a;
    table[0x80] = CPU.add_a_b;
    table[0x81] = CPU.add_a_c;
    table[0x82] = CPU.add_a_d;
    table[0x83] = CPU.add_a_e;
    table[0x84] = CPU.add_a_h;
    table[0x85] = CPU.add_a_l;
    table[0x86] = CPU.add_a_hl_ptr;
    table[0x87] = CPU.add_a_a;
    table[0x90] = CPU.sub_a_b;
    table[0x91] = CPU.sub_a_c;
    table[0x92] = CPU.sub_a_d;
    table[0x93] = CPU.sub_a_e;
    table[0x94] = CPU.sub_a_h;
    table[0x95] = CPU.sub_a_l;
    table[0x96] = CPU.sub_a_hl_ptr;
    table[0x97] = CPU.sub_a_a;
    table[0xC6] = CPU.add_a_d8;
    table[0xD6] = CPU.sub_a_d8;
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

    fn inc_b(self: *CPU) void {
        const old = self.b;
        self.b +%= 1;
        self.updateFlags(u8, old, 1, self.b, false);
    }

    fn dec_b(self: *CPU) void {
        const old = self.b;
        self.b -%= 1;
        self.updateFlags(u8, old, self.b, 1, true);
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

    fn inc_c(self: *CPU) void {
        const old = self.c;
        self.c +%= 1;
        self.updateFlags(u8, old, 1, self.c, false);
    }

    fn dec_c(self: *CPU) void {
        const old = self.c;
        self.c -%= 1;
        self.updateFlags(u8, old, 1, self.c, true);
    }

    fn inc_d(self: *CPU) void {
        const old = self.d;
        self.d +%= 1;
        self.updateFlags(u8, old, 1, self.d, false);
    }

    fn dec_d(self: *CPU) void {
        const old = self.d;
        self.d -%= 1;
        self.updateFlags(u8, old, 1, self.d, true);
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

    fn inc_e(self: *CPU) void {
        const old = self.e;
        self.e +%= 1;
        self.updateFlags(u8, old, 1, self.e, false);
    }

    fn inc_l(self: *CPU) void {
        const old = self.l;
        self.l +%= 1;
        self.updateFlags(u8, old, 1, self.l, false);
    }

    fn inc_a(self: *CPU) void {
        const old = self.a;
        self.a +%= 1;
        self.updateFlags(u8, old, 1, self.a, false);
    }

    fn dec_e(self: *CPU) void {
        const old = self.e;
        self.e -%= 1;
        self.updateFlags(u8, old, 1, self.e, true);
    }

    fn dec_l(self: *CPU) void {
        const old = self.l;
        self.l -%= 1;
        self.updateFlags(u8, old, 1, self.l, true);
    }

    fn dec_a(self: *CPU) void {
        const old = self.a;
        self.a -%= 1;
        self.updateFlags(u8, old, 1, self.a, true);
    }

    // TODO: Need interrupts and halt states to implement
    fn stop(self: *CPU) void {
        self.stopped = true;
    }

    fn inc_h(self: *CPU) void {
        const old = self.h;
        self.h +%= 1;
        self.updateFlags(u8, old, 1, self.h, false);
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

    fn dec_h(self: *CPU) void {
        const old = self.h;
        self.h -%= 1;
        self.updateFlags(u8, old, 1, self.h, true);
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

    fn add_a_b(self: *CPU) void {
        const old = self.a;
        self.a +%= self.b;
        self.updateFlags(u8, old, self.b, self.a, false);
    }

    fn add_a_c(self: *CPU) void {
        const old = self.a;
        self.a +%= self.c;
        self.updateFlags(u8, old, self.c, self.a, false);
    }

    fn add_a_d(self: *CPU) void {
        const old = self.a;
        self.a +%= self.d;
        self.updateFlags(u8, old, self.d, self.a, false);
    }

    fn add_a_e(self: *CPU) void {
        const old = self.a;
        self.a +%= self.e;
        self.updateFlags(u8, old, self.e, self.a, false);
    }

    fn add_a_h(self: *CPU) void {
        const old = self.a;
        self.a +%= self.h;
        self.updateFlags(u8, old, self.h, self.a, false);
    }

    fn add_a_l(self: *CPU) void {
        const old = self.a;
        self.a +%= self.l;
        self.updateFlags(u8, old, self.l, self.a, false);
    }

    fn add_a_hl_ptr(self: *CPU) void {
        const address = self.getHL();
        const old = self.a;
        const value = self.memory[address];
        self.a +%= value;
        self.updateFlags(u8, old, value, self.a, false);
    }

    fn add_a_a(self: *CPU) void {
        const old = self.a;
        self.a +%= self.a;
        self.updateFlags(u8, old, old, self.a, false);
    }

    fn add_a_d8(self: *CPU) void {
        const old = self.a;
        const value = self.memory[self.pc];
        self.pc += 1;
        self.a +%= value;
        self.updateFlags(u8, old, value, self.a, false);
    }

    fn sub_a_b(self: *CPU) void {
        const old = self.a;
        self.a -%= self.b;
        self.updateFlags(u8, old, self.b, self.a, true);
    }

    fn sub_a_c(self: *CPU) void {
        const old = self.a;
        self.a -%= self.c;
        self.updateFlags(u8, old, self.c, self.a, true);
    }

    fn sub_a_d(self: *CPU) void {
        const old = self.a;
        self.a -%= self.d;
        self.updateFlags(u8, old, self.d, self.a, true);
    }

    fn sub_a_e(self: *CPU) void {
        const old = self.a;
        self.a -%= self.e;
        self.updateFlags(u8, old, self.e, self.a, true);
    }

    fn sub_a_h(self: *CPU) void {
        const old = self.a;
        self.a -%= self.h;
        self.updateFlags(u8, old, self.h, self.a, true);
    }

    fn sub_a_l(self: *CPU) void {
        const old = self.a;
        self.a -%= self.l;
        self.updateFlags(u8, old, self.l, self.a, true);
    }

    fn sub_a_hl_ptr(self: *CPU) void {
        const address = self.getHL();
        const old = self.a;
        const value = self.memory[address];
        self.a -%= value;
        self.updateFlags(u8, old, value, self.a, true);
    }

    fn sub_a_a(self: *CPU) void {
        const old = self.a;
        self.a -%= self.a;
        self.updateFlags(u8, old, old, self.a, true);
    }

    fn sub_a_d8(self: *CPU) void {
        const old = self.a;
        const value = self.memory[self.pc];
        self.pc += 1;
        self.a -%= value;
        self.updateFlags(u8, old, value, self.a, true);
    }

    // LD r,r instructions
    fn ld_b_b(self: *CPU) void {
        _ = self;
    }
    fn ld_b_c(self: *CPU) void {
        self.b = self.c;
    }
    fn ld_b_d(self: *CPU) void {
        self.b = self.d;
    }
    fn ld_b_e(self: *CPU) void {
        self.b = self.e;
    }
    fn ld_b_h(self: *CPU) void {
        self.b = self.h;
    }
    fn ld_b_l(self: *CPU) void {
        self.b = self.l;
    }
    fn ld_b_hl_ptr(self: *CPU) void {
        self.b = self.memory[self.getHL()];
    }
    fn ld_b_a(self: *CPU) void {
        self.b = self.a;
    }

    fn ld_c_b(self: *CPU) void {
        self.c = self.b;
    }
    fn ld_c_c(self: *CPU) void {
        _ = self;
    }
    fn ld_c_d(self: *CPU) void {
        self.c = self.d;
    }
    fn ld_c_e(self: *CPU) void {
        self.c = self.e;
    }
    fn ld_c_h(self: *CPU) void {
        self.c = self.h;
    }
    fn ld_c_l(self: *CPU) void {
        self.c = self.l;
    }
    fn ld_c_hl_ptr(self: *CPU) void {
        self.c = self.memory[self.getHL()];
    }
    fn ld_c_a(self: *CPU) void {
        self.c = self.a;
    }

    fn ld_d_b(self: *CPU) void {
        self.d = self.b;
    }
    fn ld_d_c(self: *CPU) void {
        self.d = self.c;
    }
    fn ld_d_d(self: *CPU) void {
        _ = self;
    }
    fn ld_d_e(self: *CPU) void {
        self.d = self.e;
    }
    fn ld_d_h(self: *CPU) void {
        self.d = self.h;
    }
    fn ld_d_l(self: *CPU) void {
        self.d = self.l;
    }
    fn ld_d_hl_ptr(self: *CPU) void {
        self.d = self.memory[self.getHL()];
    }
    fn ld_d_a(self: *CPU) void {
        self.d = self.a;
    }

    fn ld_e_b(self: *CPU) void {
        self.e = self.b;
    }
    fn ld_e_c(self: *CPU) void {
        self.e = self.c;
    }
    fn ld_e_d(self: *CPU) void {
        self.e = self.d;
    }
    fn ld_e_e(self: *CPU) void {
        _ = self;
    }
    fn ld_e_h(self: *CPU) void {
        self.e = self.h;
    }
    fn ld_e_l(self: *CPU) void {
        self.e = self.l;
    }
    fn ld_e_hl_ptr(self: *CPU) void {
        self.e = self.memory[self.getHL()];
    }
    fn ld_e_a(self: *CPU) void {
        self.e = self.a;
    }

    fn ld_h_b(self: *CPU) void {
        self.h = self.b;
    }
    fn ld_h_c(self: *CPU) void {
        self.h = self.c;
    }
    fn ld_h_d(self: *CPU) void {
        self.h = self.d;
    }
    fn ld_h_e(self: *CPU) void {
        self.h = self.e;
    }
    fn ld_h_h(self: *CPU) void {
        _ = self;
    }
    fn ld_h_l(self: *CPU) void {
        self.h = self.l;
    }
    fn ld_h_hl_ptr(self: *CPU) void {
        self.h = self.memory[self.getHL()];
    }
    fn ld_h_a(self: *CPU) void {
        self.h = self.a;
    }

    fn ld_l_b(self: *CPU) void {
        self.l = self.b;
    }
    fn ld_l_c(self: *CPU) void {
        self.l = self.c;
    }
    fn ld_l_d(self: *CPU) void {
        self.l = self.d;
    }
    fn ld_l_e(self: *CPU) void {
        self.l = self.e;
    }
    fn ld_l_h(self: *CPU) void {
        self.l = self.h;
    }
    fn ld_l_l(self: *CPU) void {
        _ = self;
    }
    fn ld_l_hl_ptr(self: *CPU) void {
        self.l = self.memory[self.getHL()];
    }
    fn ld_l_a(self: *CPU) void {
        self.l = self.a;
    }

    fn ld_hl_ptr_b(self: *CPU) void {
        self.memory[self.getHL()] = self.b;
    }
    fn ld_hl_ptr_c(self: *CPU) void {
        self.memory[self.getHL()] = self.c;
    }
    fn ld_hl_ptr_d(self: *CPU) void {
        self.memory[self.getHL()] = self.d;
    }
    fn ld_hl_ptr_e(self: *CPU) void {
        self.memory[self.getHL()] = self.e;
    }
    fn ld_hl_ptr_h(self: *CPU) void {
        self.memory[self.getHL()] = self.h;
    }
    fn ld_hl_ptr_l(self: *CPU) void {
        self.memory[self.getHL()] = self.l;
    }
    fn ld_hl_ptr_a(self: *CPU) void {
        self.memory[self.getHL()] = self.a;
    }

    fn ld_a_b(self: *CPU) void {
        self.a = self.b;
    }
    fn ld_a_c(self: *CPU) void {
        self.a = self.c;
    }
    fn ld_a_d(self: *CPU) void {
        self.a = self.d;
    }
    fn ld_a_e(self: *CPU) void {
        self.a = self.e;
    }
    fn ld_a_h(self: *CPU) void {
        self.a = self.h;
    }
    fn ld_a_l(self: *CPU) void {
        self.a = self.l;
    }
    fn ld_a_hl_ptr(self: *CPU) void {
        self.a = self.memory[self.getHL()];
    }
    fn ld_a_a(self: *CPU) void {
        _ = self;
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

    fn and_b(self: *CPU) void {
        self.a &= self.b;
        self.checkFlag(.z, self.a == 0);
        self.unsetFlag(.n);
        self.setFlag(.h);
        self.unsetFlag(.c);
    }

    fn and_c(self: *CPU) void {
        self.a &= self.c;
        self.checkFlag(.z, self.a == 0);
        self.unsetFlag(.n);
        self.setFlag(.h);
        self.unsetFlag(.c);
    }

    fn and_d(self: *CPU) void {
        self.a &= self.d;
        self.checkFlag(.z, self.a == 0);
        self.unsetFlag(.n);
        self.setFlag(.h);
        self.unsetFlag(.c);
    }

    fn and_e(self: *CPU) void {
        self.a &= self.e;
        self.checkFlag(.z, self.a == 0);
        self.unsetFlag(.n);
        self.setFlag(.h);
        self.unsetFlag(.c);
    }

    fn and_h(self: *CPU) void {
        self.a &= self.h;
        self.checkFlag(.z, self.a == 0);
        self.unsetFlag(.n);
        self.setFlag(.h);
        self.unsetFlag(.c);
    }

    fn and_l(self: *CPU) void {
        self.a &= self.l;
        self.checkFlag(.z, self.a == 0);
        self.unsetFlag(.n);
        self.setFlag(.h);
        self.unsetFlag(.c);
    }

    fn and_hl_ptr(self: *CPU) void {
        self.a &= self.memory[self.getHL()];
        self.checkFlag(.z, self.a == 0);
        self.unsetFlag(.n);
        self.setFlag(.h);
        self.unsetFlag(.c);
    }

    fn and_a(self: *CPU) void {
        self.a &= self.a;
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

    fn or_b(self: *CPU) void {
        self.a |= self.b;
        self.checkFlag(.z, self.a == 0);
        self.unsetFlag(.n);
        self.unsetFlag(.h);
        self.unsetFlag(.c);
    }

    fn or_c(self: *CPU) void {
        self.a |= self.c;
        self.checkFlag(.z, self.a == 0);
        self.unsetFlag(.n);
        self.unsetFlag(.h);
        self.unsetFlag(.c);
    }

    fn or_d(self: *CPU) void {
        self.a |= self.d;
        self.checkFlag(.z, self.a == 0);
        self.unsetFlag(.n);
        self.unsetFlag(.h);
        self.unsetFlag(.c);
    }

    fn or_e(self: *CPU) void {
        self.a |= self.e;
        self.checkFlag(.z, self.a == 0);
        self.unsetFlag(.n);
        self.unsetFlag(.h);
        self.unsetFlag(.c);
    }

    fn or_h(self: *CPU) void {
        self.a |= self.h;
        self.checkFlag(.z, self.a == 0);
        self.unsetFlag(.n);
        self.unsetFlag(.h);
        self.unsetFlag(.c);
    }

    fn or_l(self: *CPU) void {
        self.a |= self.l;
        self.checkFlag(.z, self.a == 0);
        self.unsetFlag(.n);
        self.unsetFlag(.h);
        self.unsetFlag(.c);
    }

    fn or_hl_ptr(self: *CPU) void {
        self.a |= self.memory[self.getHL()];
        self.checkFlag(.z, self.a == 0);
        self.unsetFlag(.n);
        self.unsetFlag(.h);
        self.unsetFlag(.c);
    }

    fn or_a(self: *CPU) void {
        self.a |= self.a;
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

    fn xor_b(self: *CPU) void {
        self.a ^= self.b;
        self.checkFlag(.z, self.a == 0);
        self.unsetFlag(.n);
        self.unsetFlag(.h);
        self.unsetFlag(.c);
    }

    fn xor_c(self: *CPU) void {
        self.a ^= self.c;
        self.checkFlag(.z, self.a == 0);
        self.unsetFlag(.n);
        self.unsetFlag(.h);
        self.unsetFlag(.c);
    }

    fn xor_d(self: *CPU) void {
        self.a ^= self.d;
        self.checkFlag(.z, self.a == 0);
        self.unsetFlag(.n);
        self.unsetFlag(.h);
        self.unsetFlag(.c);
    }

    fn xor_e(self: *CPU) void {
        self.a ^= self.e;
        self.checkFlag(.z, self.a == 0);
        self.unsetFlag(.n);
        self.unsetFlag(.h);
        self.unsetFlag(.c);
    }

    fn xor_h(self: *CPU) void {
        self.a ^= self.h;
        self.checkFlag(.z, self.a == 0);
        self.unsetFlag(.n);
        self.unsetFlag(.h);
        self.unsetFlag(.c);
    }

    fn xor_l(self: *CPU) void {
        self.a ^= self.l;
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

    fn xor_a(self: *CPU) void {
        self.a ^= self.a;
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

    fn unknown_opcode(self: *CPU) void {
        _ = self;
        @panic("Unknown opcode");
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

