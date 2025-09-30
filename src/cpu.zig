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

    fn ld_bc_d16(self: *CPU) void {
        self.c = self.memory[self.pc];
        self.pc += 1;
        self.b = self.memory[self.pc];
        self.pc += 1;
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
        self.setIncFlags8(old, self.b);
    }

    fn dec_b(self: *CPU) void {
        const old = self.b;
        self.b -%= 1;
        self.setDecFlags8(old, self.b);
    }

    fn add_hl_bc(self: *CPU) void {
        const hl = self.getHL();
        const bc = self.getBC();
        const result = hl +% bc;
        self.setAddFlags16(hl, bc, result);
        self.setHL(result);
    }

    fn dec_bc(self: *CPU) void {
        self.setBC(self.getBC() -% 1);
    }

    fn inc_c(self: *CPU) void {
        const old = self.c;
        self.c +%= 1;
        self.setIncFlags8(old, self.c);
    }

    fn dec_c(self: *CPU) void {
        const old = self.c;
        self.c -%= 1;
        self.setDecFlags8(old, self.c);
    }

    fn ld_de_d16(self: *CPU) void {
        self.d = self.memory[self.pc];
        self.pc += 1;
        self.e = self.memory[self.pc];
        self.pc += 1;
    }

    fn inc_d(self: *CPU) void {
        const old = self.d;
        self.d +%= 1;
        self.setIncFlags8(old, self.d);
    }

    fn dec_d(self: *CPU) void {
        const old = self.d;
        self.d -%= 1;
        self.setDecFlags8(old, self.d);
    }

    fn add_hl_de(self: *CPU) void {
        const hl = self.getHL();
        const de = self.getDE();
        const result = hl +% de;
        self.setAddFlags16(hl, de, result);
        self.setHL(result);
    }

    fn dec_de(self: *CPU) void {
        self.setDE(self.getDE() - 1);
    }

    fn rlca(self: *CPU) void {
        const bit7 = (self.a >> 7) & 1;
        self.a = (self.a << 1) | bit7;

        self.unsetZFlag();
        self.unsetNFlag();
        self.unsetHFlag();
        if (bit7 != 0) {
            self.setCFlag();
        } else {
            self.unsetCFlag();
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

        self.unsetZFlag();
        self.unsetNFlag();
        self.unsetHFlag();
        if (bit0 != 0) {
            self.setCFlag();
        } else {
            self.unsetCFlag();
        }
    }

    fn ld_a16_sp(self: *CPU) void {
        const address: u16 = std.mem.readInt(u16, self.memory[self.pc..][0..2], .little);
        self.pc += 2;
        std.mem.writeInt(u16, self.memory[address..][0..2], self.sp, .little);
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

    fn setZFlag(self: *CPU) void {
        self.f |= (1 << 7);
    }

    fn setNFlag(self: *CPU) void {
        self.f |= (1 << 6);
    }

    fn setHFlag(self: *CPU) void {
        self.f |= (1 << 5);
    }

    fn setCFlag(self: *CPU) void {
        self.f |= (1 << 4);
    }

    fn unsetZFlag(self: *CPU) void {
        self.f &= ~(@as(u8, 1) << 7);
    }

    fn unsetNFlag(self: *CPU) void {
        self.f &= ~(@as(u8, 1) << 6);
    }

    fn unsetHFlag(self: *CPU) void {
        self.f &= ~(@as(u8, 1) << 5);
    }

    fn unsetCFlag(self: *CPU) void {
        self.f &= ~(@as(u8, 1) << 4);
    }

    fn checkZFlag(self: *CPU, value: u8) void {
        if (value == 0) {
            self.setZFlag();
        } else {
            self.unsetZFlag();
        }
    }

    fn setAddFlags8(self: *CPU, a: u8, b: u8, result: u8) void {
        if (result == 0) {
            self.setZFlag();
        } else {
            self.unsetZFlag();
        }
        self.unsetNFlag();
        if ((a & 0x0F) + (b & 0x0F) > 0x0F) {
            self.setHFlag();
        } else {
            self.unsetHFlag();
        }
        if (result < a) {
            self.setCFlag();
        } else {
            self.unsetCFlag();
        }
    }

    fn setSubFlags8(self: *CPU, a: u8, b: u8, result: u8) void {
        if (result == 0) {
            self.setZFlag();
        } else {
            self.unsetZFlag();
        }
        self.setNFlag();
        if ((a & 0x0F) < (b & 0x0F)) {
            self.setHFlag();
        } else {
            self.unsetHFlag();
        }
        if (a < b) {
            self.setCFlag();
        } else {
            self.unsetCFlag();
        }
    }

    fn setAddFlags16(self: *CPU, a: u16, b: u16, result: u16) void {
        self.unsetNFlag();
        if ((a & 0x0FFF) + (b & 0x0FFF) > 0x0FFF) {
            self.setHFlag();
        } else {
            self.unsetHFlag();
        }
        if (result < a) {
            self.setCFlag();
        } else {
            self.unsetCFlag();
        }
    }

    fn setIncFlags8(self: *CPU, old_value: u8, new_value: u8) void {
        if (new_value == 0) {
            self.setZFlag();
        } else {
            self.unsetZFlag();
        }
        self.unsetNFlag();
        if ((old_value & 0x0F) == 0x0F) {
            self.setHFlag();
        } else {
            self.unsetHFlag();
        }
    }

    fn setDecFlags8(self: *CPU, old_value: u8, new_value: u8) void {
        if (new_value == 0) {
            self.setZFlag();
        } else {
            self.unsetZFlag();
        }
        self.setNFlag();
        if ((old_value & 0x0F) == 0x00) {
            self.setHFlag();
        } else {
            self.unsetHFlag();
        }
    }
};

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
    table[0x1E] = CPU.ld_e_d8;
    table[0x1F] = CPU.rra;
    table[0x22] = CPU.ld_hlplus_a;
    table[0x26] = CPU.ld_h_d8;
    table[0x2A] = CPU.ld_a_hlplus;
    table[0x2E] = CPU.ld_l_d8;
    table[0x32] = CPU.ld_hlminus_a;
    table[0x3A] = CPU.ld_a_hlminus;
    break :blk table;
};
