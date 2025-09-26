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
        self.unsetNFlag();
        self.b +%= 1;
        self.checkZFlag(&self.b);
        self.checkHFlag(&self.b);
    }

    fn dec_b(self: *CPU) void {
        self.setNFlag();
        self.checkHFlag(&self.b);
        self.b -%= 1;
        self.checkZFlag(&self.b);
    }

    fn add_hl_bc(self: *CPU) void {
        self.unsetNFlag();
        const hl = self.getHL();
        const bc = self.getBC();
        const result = hl +% bc;

        if (result < hl) {
            self.setCFlag();
        } else {
            self.unsetCFlag();
        }

        if ((hl & 0x0FFF) + (bc & 0x0FFF) > 0x0FFF) {
            self.setHFlag();
        } else {
            self.unsetHFlag();
        }

        self.setHL(result);
    }

    fn dec_bc(self: *CPU) void {
        self.setBC(self.getBC() -% 1);
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

    fn checkZFlag(self: *CPU, reg: *u8) void {
        if (reg.* == 0) {
            self.setZFlag();
        } else {
            self.unsetZFlag();
        }
    }

    fn checkHFlag(self: *CPU, reg: *u8) void {
        if ((reg.* & 0x0F) == 0) {
            self.setHFlag();
        } else {
            self.unsetHFlag();
        }
    }

    fn checkCFlag(self: *CPU, reg: *u8) void {
        if (reg.* == 0) {
            self.setCFlag();
        } else {
            self.unsetCFlag();
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
    table[0x09] = CPU.add_hl_bc;
    table[0x0A] = CPU.ld_a_bc;
    table[0x0B] = CPU.dec_bc;
    table[0x0E] = CPU.ld_c_d8;
    table[0x12] = CPU.ld_de_a;
    table[0x16] = CPU.ld_d_d8;
    table[0x1A] = CPU.ld_a_de;
    table[0x1E] = CPU.ld_e_d8;
    table[0x22] = CPU.ld_hlplus_a;
    table[0x26] = CPU.ld_h_d8;
    table[0x2A] = CPU.ld_a_hlplus;
    table[0x2E] = CPU.ld_l_d8;
    table[0x32] = CPU.ld_hlminus_a;
    table[0x3A] = CPU.ld_a_hlminus;
    break :blk table;
};
