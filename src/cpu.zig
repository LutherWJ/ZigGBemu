const std = @import("std");

const InstructionFn = *const fn (*CPU) void;

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

    fn ld_bc_a(self: *CPU) void {
        const address = self.getBC();
        self.memory[address] = self.a;
    }

    fn ld_b_d8(self: *CPU) void {
        self.b = self.memory[self.pc];
        self.pc += 1;
    }

    fn ld_a_bc(self: *CPU) void {
        const address = self.getBC();
        self.a = self.memory[address];
    }

    fn ld_c_d8(self: *CPU) void {
        self.c = self.memory[self.pc];
        self.pc += 1;
    }

    fn ld_de_a(self: *CPU) void {
        const address = self.getDE();
        self.memory[address] = self.a;
    }

    fn ld_d_d8(self: *CPU) void {
        self.d = self.memory[self.pc];
        self.pc += 1;
    }

    fn ld_a_de(self: *CPU) void {
        const address = self.getDE();
        self.a = self.memory[address];
    }

    fn ld_e_d8(self: *CPU) void {
        self.e = self.memory[self.pc];
        self.pc += 1;
    }

    fn ld_hlplus_a(self: *CPU) void {
        const address = self.getHL();
        self.memory[address] = self.a;
        self.setHL(self.getHL() + 1);
    }

    fn ld_h_d8(self: *CPU) void {
        self.h = self.memory[self.pc];
        self.pc += 1;
    }

    fn ld_a_hlplus(self: *CPU) void {
        const address = self.getHL();
        self.a = self.memory[address];
        self.setHL(self.getHL() + 1);
    }

    fn ld_l_d8(self: *CPU) void {
        self.l = self.memory[self.pc];
        self.pc += 1;
    }

    fn ld_hlminus_a(self: *CPU) void {
        const address = self.getHL();
        self.memory[address] = self.a;
        self.setHL(self.getHL() - 1);
    }

    fn ld_a_hlminus(self: *CPU) void {
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

    fn ld_a16_sp(self: *CPU) void {
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

const Flag = enum(u3) { z = 7, n = 6, h = 5, c = 4 };

// Instruction table matching opcodes to their corresponding functions
const instruction_table = blk: {
    var table = [_]InstructionFn{CPU.unknown_opcode} ** 256;
    table[0x00] = CPU.nop;
    table[0x01] = CPU.ld_bc_d16;
    table[0x02] = CPU.ld_bc_a;
    table[0x03] = CPU.inc_bc;
    table[0x04] = CPU.inc_b;
    table[0x05] = CPU.dec_b;
    table[0x06] = CPU.ld_b_d8;
    table[0x07] = CPU.rlca;
    table[0x08] = CPU.ld_a16_sp;
    table[0x09] = CPU.add_hl_bc;
    table[0x0A] = CPU.ld_a_bc;
    table[0x0B] = CPU.dec_bc;
    table[0x0C] = CPU.inc_c;
    table[0x0D] = CPU.dec_c;
    table[0x0E] = CPU.ld_c_d8;
    table[0x0F] = CPU.rrca;
    table[0x11] = CPU.ld_de_d16;
    table[0x12] = CPU.ld_de_a;
    table[0x14] = CPU.inc_d;
    table[0x15] = CPU.dec_d;
    table[0x16] = CPU.ld_d_d8;
    table[0x17] = CPU.rla;
    table[0x19] = CPU.add_hl_de;
    table[0x1A] = CPU.ld_a_de;
    table[0x1B] = CPU.dec_de;
    table[0x1C] = CPU.inc_e;
    table[0x1D] = CPU.dec_e;
    table[0x1E] = CPU.ld_e_d8;
    table[0x1F] = CPU.rra;
    table[0x18] = CPU.jr_r8;
    table[0x20] = CPU.jr_nz_r8;
    table[0x21] = CPU.ld_hl_d16;
    table[0x22] = CPU.ld_hlplus_a;
    table[0x26] = CPU.ld_h_d8;
    table[0x28] = CPU.jr_z_r8;
    table[0x2A] = CPU.ld_a_hlplus;
    table[0x2C] = CPU.inc_l;
    table[0x2D] = CPU.dec_l;
    table[0x2E] = CPU.ld_l_d8;
    table[0x30] = CPU.jr_nc_r8;
    table[0x32] = CPU.ld_hlminus_a;
    table[0x38] = CPU.jr_c_r8;
    table[0x3A] = CPU.ld_a_hlminus;
    table[0x3C] = CPU.inc_a;
    table[0x3D] = CPU.dec_a;
    break :blk table;
};
