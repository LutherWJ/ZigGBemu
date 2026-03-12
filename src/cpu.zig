const std = @import("std");
const MMU = @import("mmu.zig").MMU;
const InterruptBit = @import("mmu.zig").InterruptBit;

const InstructionFn = *const fn (*CPU) void;
const Flag = enum(u3) { z = 7, n = 6, h = 5, c = 4 };
const Register = enum { a, f, b, c, d, e, h, l };
const U16Register = enum { bc, de, hl, af, sp, pc };
const Conditions = enum { always, nz, z, nc, c }; // Used for conditional instructions like CALL or JP
const ImeState = enum { disabled, enable_pending, enabled }; // interrupts are sometimes delayed one cycle
const SourceType = enum { register, hl_ptr, immediate };

const HIGH_RAM_BASE_ADDRESS: u16 = 0xFF00;
const BIT0 = 1;
const BIT7 = 0b10000000;
const BIT15 = 0x8000;

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

            if (cpu.checkConditional(cond)) {
                const offset_i16: i16 = offset;
                const offset_u16: u16 = @bitCast(offset_i16);
                cpu.pc +%= offset_u16;
            }
        }
    }.f;
}

fn makeJumpAbsolute(comptime cond: Conditions) InstructionFn {
    return struct {
        fn f(cpu: *CPU) void {
            const address: u16 = cpu.memory.read(cpu.pc, u16);
            cpu.pc +%= 2;

            if (cpu.checkConditional(cond)) {
                cpu.pc = address;
            }
        }
    }.f;
}

fn makeCall(comptime cond: Conditions) InstructionFn {
    return struct {
        fn f(cpu: *CPU) void {
            const address: u16 = cpu.memory.read(cpu.pc, u16);
            cpu.pc +%= 2;

            if (cpu.checkConditional(cond)) {
                cpu.pushU16(cpu.pc);
                cpu.pc = address;
            }
        }
    }.f;
}

fn makeRet(comptime cond: Conditions) InstructionFn {
    return struct {
        fn f(cpu: *CPU) void {
            if (cpu.checkConditional(cond)) {
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

fn makeRLC(comptime regName: Register) InstructionFn {
    return struct {
        fn f(cpu: *CPU) void {
            const reg = cpu.getRegister(regName);
            const bit7 = (reg.* >> 7) & 1;
            reg.* = (reg.* << 1) | bit7;

            cpu.checkFlag(.z, reg.* == 0);
            cpu.checkFlag(.c, bit7 != 0);
            cpu.unsetFlag(.h);
            cpu.unsetFlag(.n);
        }
    }.f;
}

fn makeRLC16(comptime regName: U16Register) InstructionFn {
    return struct {
        fn f(cpu: *CPU) void {
            const reg = cpu.getU16Register(regName);
            const bit15 = (reg >> 15) & 1;
            const result = (reg << 1) | bit15;
            cpu.setU16Register(regName, result);

            cpu.checkFlag(.z, result == 0);
            cpu.checkFlag(.c, bit15 != 0);
            cpu.unsetFlag(.h);
            cpu.unsetFlag(.n);
        }
    }.f;
}

fn makeRRC(comptime regName: Register) InstructionFn {
    return struct {
        fn f(cpu: *CPU) void {
            const reg = cpu.getRegister(regName);
            const bit0 = reg.* & 1;
            reg.* = (reg.* >> 1) | (bit0 << 7);

            cpu.checkFlag(.z, reg.* == 0);
            cpu.checkFlag(.c, bit0 != 0);
            cpu.unsetFlag(.h);
            cpu.unsetFlag(.n);
        }
    }.f;
}

fn makeRRC16(comptime regName: U16Register) InstructionFn {
    return struct {
        fn f(cpu: *CPU) void {
            const reg = cpu.getU16Register(regName);
            const bit0 = reg & 1;
            const result = (reg >> 1) | (bit0 << 15);
            cpu.setU16Register(regName, result);

            cpu.checkFlag(.z, result == 0);
            cpu.checkFlag(.c, bit0 != 0);
            cpu.unsetFlag(.h);
            cpu.unsetFlag(.n);
        }
    }.f;
}

fn makeRR(comptime regName: Register) InstructionFn {
    return struct {
        fn f(cpu: *CPU) void {
            const reg = cpu.getRegister(regName);
            const bit0 = reg.* & 1;
            const c_flag = cpu.readFlag(.c);
            reg.* >>= 1;
            reg.* |= @as(u8, @intFromBool(c_flag)) << 7;

            cpu.checkFlag(.c, bit0 != 0);
            cpu.checkFlag(.z, reg.* == 0);
            cpu.unsetFlag(.h);
            cpu.unsetFlag(.n);
        }
    }.f;
}

fn makeRL(comptime regName: Register) InstructionFn {
    return struct {
        fn f(cpu: *CPU) void {
            const reg = cpu.getRegister(regName);
            const bit7 = (reg.*) == BIT7;
            const c_flag = cpu.readFlag(.c);
            reg.* <<= 1;
            reg.* |= @intFromBool(c_flag);

            cpu.checkFlag(.c, bit7);
            cpu.checkFlag(.z, reg.* == 0);
            cpu.unsetFlag(.h);
            cpu.unsetFlag(.n);
        }
    }.f;
}

fn makeRL16(comptime regName: U16Register) InstructionFn {
    return struct {
        fn f(cpu: *CPU) void {
            const reg = cpu.getU16Register(regName);
            const bit15 = (reg >> 15) & 1;
            const c_flag = cpu.readFlag(.c);
            const result = (reg << 1) | @intFromBool(c_flag);
            cpu.setU16Register(regName, result);

            cpu.checkFlag(.c, bit15 != 0);
            cpu.checkFlag(.z, result == 0);
            cpu.unsetFlag(.h);
            cpu.unsetFlag(.n);
        }
    }.f;
}

fn makeRR16(comptime regName: U16Register) InstructionFn {
    return struct {
        fn f(cpu: *CPU) void {
            const reg = cpu.getU16Register(regName);
            const bit0 = reg & 1;
            const c_flag = cpu.readFlag(.c);
            const result = (reg >> 1) | (@as(u16, @intFromBool(c_flag)) << 15);
            cpu.setU16Register(regName, result);

            cpu.checkFlag(.c, bit0 != 0);
            cpu.checkFlag(.z, result == 0);
            cpu.unsetFlag(.h);
            cpu.unsetFlag(.n);
        }
    }.f;
}

fn makeSLA(comptime regName: Register) InstructionFn {
    return struct {
        fn f(cpu: *CPU) void {
            const reg = cpu.getRegister(regName);
            const bit0 = reg.* & BIT0;
            const bit7 = (reg.* & BIT7) == BIT7;
            reg.* <<= 1;
            reg.* |= bit0;

            cpu.checkFlag(.c, bit7);
            cpu.checkFlag(.z, reg.* == 0);
            cpu.unsetFlag(.h);
            cpu.unsetFlag(.n);
        }
    }.f;
}

fn makeSRA(comptime regName: Register) InstructionFn {
    return struct {
        fn f(cpu: *CPU) void {
            const reg = cpu.getRegister(regName);
            const bit0 = (reg.* & BIT0) == BIT0;
            const bit7 = reg.* & BIT7;
            reg.* >>= 1;
            reg.* |= bit7;

            cpu.checkFlag(.c, bit0);
            cpu.checkFlag(.z, reg.* == 0);
            cpu.unsetFlag(.h);
            cpu.unsetFlag(.n);
        }
    }.f;
}

fn makeSRL(comptime regName: Register) InstructionFn {
    return struct {
        fn f(cpu: *CPU) void {
            const reg = cpu.getRegister(regName);
            const bit0 = (reg.* & BIT0) == BIT0;
            reg.* >>= 1;

            cpu.checkFlag(.c, bit0);
            cpu.checkFlag(.z, reg.* == 0);
            cpu.unsetFlag(.h);
            cpu.unsetFlag(.n);
        }
    }.f;
}

fn makeSRL16(comptime regName: U16Register) InstructionFn {
    return struct {
        fn f(cpu: *CPU) void {
            const reg = cpu.getU16Register(regName);
            const bit0 = reg & 1;
            const result = reg >> 1;
            cpu.setU16Register(regName, result);

            cpu.checkFlag(.c, bit0 != 0);
            cpu.checkFlag(.z, result == 0);
            cpu.unsetFlag(.h);
            cpu.unsetFlag(.n);
        }
    }.f;
}

fn makeSLA16(comptime regName: U16Register) InstructionFn {
    return struct {
        fn f(cpu: *CPU) void {
            const reg = cpu.getU16Register(regName);
            const bit15 = (reg & BIT15) == BIT15;
            const result = reg << 1;
            cpu.setU16Register(regName, result);

            cpu.checkFlag(.c, bit15);
            cpu.checkFlag(.z, result == 0);
            cpu.unsetFlag(.h);
            cpu.unsetFlag(.n);
        }
    }.f;
}

fn makeSRA16(comptime regName: U16Register) InstructionFn {
    return struct {
        fn f(cpu: *CPU) void {
            const reg = cpu.getU16Register(regName);
            const bit0 = (reg & BIT0) == BIT0;
            const bit15 = reg & BIT15;
            const result = (reg >> 1) | bit15;
            cpu.setU16Register(regName, result);

            cpu.checkFlag(.c, bit0);
            cpu.checkFlag(.z, result == 0);
            cpu.unsetFlag(.h);
            cpu.unsetFlag(.n);
        }
    }.f;
}

fn makeBIT(comptime bit_num: u3, comptime src_type: SourceType, comptime reg: ?Register) InstructionFn {
    return struct {
        fn f(cpu: *CPU) void {
            const dst = fetchSource(cpu, src_type, reg);
            cpu.checkFlag(.z, (dst & (@as(u8, 1) << bit_num)) == 0);
            cpu.unsetFlag(.n);
            cpu.setFlag(.h);
        }
    }.f;
}

fn makeRES(comptime bit_num: u3, comptime src_type: SourceType, comptime reg: ?Register) InstructionFn {
    return struct {
        fn f(cpu: *CPU) void {
            switch (src_type) {
                .register => {
                    cpu.getRegister(reg.?).* &= ~(@as(u8, 1) << bit_num);
                },
                .hl_ptr => {
                    const address = cpu.getU16Register(.hl);
                    var val = cpu.memory.read(address, u8);
                    val &= ~(@as(u8, 1) << bit_num);
                    cpu.memory.write(address, val);
                },
                .immediate => @compileError("Immediate SourceType is not supported on RES instructions."),
            }
        }
    }.f;
}

fn makeSET(comptime bit_num: u3, comptime src_type: SourceType, comptime reg: ?Register) InstructionFn {
    return struct {
        fn f(cpu: *CPU) void {
            switch (src_type) {
                .hl_ptr => {
                    const address = cpu.getU16Register(.hl);
                    var val = cpu.memory.read(address, u8);
                    val |= (@as(u8, 1) << bit_num);
                    cpu.memory.write(address, val);
                },
                .register => cpu.getRegister(reg.?).* |= (@as(u8, 1) << bit_num),
                .immediate => @compileError("Immediate SourceType is not supported on SET instructions."),
            }
        }
    }.f;
}

fn makeSWAP(comptime src: SourceType, reg_name: ?Register) InstructionFn {
    return struct {
        fn f(cpu: *CPU) void {
            switch (src) {
                .register => {
                    const reg = cpu.getRegister(reg_name.?);
                    const high = reg.* >> 4;
                    reg.* = (reg.* << 4) | high;
                    cpu.checkFlag(.z, reg.* == 0);
                },
                .hl_ptr => {
                    const address = cpu.getU16Register(.hl);
                    const val = cpu.memory.read(address, u8);
                    const high = val >> 4;
                    cpu.memory.write(address, (val << 4) | high);
                    cpu.checkFlag(.z, val == 0);
                },
                .immediate => @compileError("Immediate SourceType is not supported on SWAP instructions."),
            }

            cpu.unsetFlag(.c);
            cpu.unsetFlag(.n);
            cpu.unsetFlag(.h);
        }
    }.f;
}
// Instruction table matching opcodes to their corresponding functions
const instruction_table = blk: {
    var table = [_]InstructionFn{CPU.unknown_opcode} ** 256;

    const getReg = struct {
        fn f(comptime i: usize) ?Register {
            return switch (i) {
                0 => .b,
                1 => .c,
                2 => .d,
                3 => .e,
                4 => .h,
                5 => .l,
                6 => null,
                7 => .a,
                else => unreachable,
            };
        }
    }.f;

    table[0x00] = CPU.nop;
    table[0x07] = CPU.rlca;
    table[0x08] = CPU.ld_a16_ptr_sp;
    table[0x0F] = CPU.rrca;
    table[0x10] = CPU.stop;
    table[0x17] = CPU.rla;
    table[0x1F] = CPU.rra;
    table[0x22] = CPU.ld_hl_ptr_plus_a;
    table[0x27] = CPU.daa;
    table[0x2A] = CPU.ld_a_hl_ptr_plus;
    table[0x2F] = CPU.cpl;
    table[0x31] = CPU.ld_sp_d16;
    table[0x32] = CPU.ld_hl_ptr_minus_a;
    table[0x34] = CPU.inc_hl_ptr;
    table[0x35] = CPU.dec_hl_ptr;
    table[0x36] = CPU.ld_hl_ptr_d8;
    table[0x37] = CPU.scf;
    table[0x3A] = CPU.ld_a_hl_ptr_minus;
    table[0x3F] = CPU.ccf;
    table[0x76] = CPU.halt;
    table[0xC1] = makePop(.bc);
    table[0xC5] = makePush(.bc);
    table[0xC6] = makeAdd(.immediate, null);
    table[0xC9] = makeRet(.always);
    table[0xCB] = CPU.cb_prefix;
    table[0xCD] = makeCall(.always);
    table[0xCE] = makeAdc(.immediate, null);
    table[0xD1] = makePop(.de);
    table[0xD5] = makePush(.de);
    table[0xD6] = makeSub(.immediate, null);
    table[0xD9] = CPU.reti;
    table[0xDE] = makeSbc(.immediate, null);
    table[0xE0] = CPU.ld_a8_a;
    table[0xE1] = makePop(.hl);
    table[0xE2] = CPU.ld_c_a;
    table[0xE5] = makePush(.hl);
    table[0xE6] = makeAnd(.immediate, null);
    table[0xE8] = CPU.add_sp_r8;
    table[0xE9] = CPU.jp_hl;
    table[0xEA] = CPU.ld_a16_a;
    table[0xEE] = makeXor(.immediate, null);
    table[0xF0] = CPU.ldh_a_a8;
    table[0xF1] = makePop(.af);
    table[0xF2] = CPU.ldh_a_c;
    table[0xF3] = CPU.di;
    table[0xF5] = makePush(.af);
    table[0xF6] = makeOr(.immediate, null);
    table[0xF8] = CPU.ld_hl_sp_r8;
    table[0xF9] = CPU.ld_sp_hl;
    table[0xFA] = CPU.ld_a_a16;
    table[0xFB] = CPU.ei;
    table[0xFE] = makeCp(.immediate, null);

    // 16-bit LD, INC, DEC, ADD
    const rr_regs = [_]U16Register{ .bc, .de, .hl, .sp };
    for (rr_regs, 0..) |rr, i| {
        if (rr != .sp) table[0x01 + i * 16] = makeLdU16RegD16(rr);
        table[0x03 + i * 16] = makeIncU16Reg(rr);
        table[0x0B + i * 16] = makeDecU16Reg(rr);
        table[0x09 + i * 16] = makeAddHl(rr);
    }

    // 8-bit INC, DEC, LD d8
    for (0..8) |i| {
        const reg = getReg(i);
        if (reg) |r| {
            table[i * 8 + 4] = makeIncReg(r);
            table[i * 8 + 5] = makeDecReg(r);
            table[i * 8 + 6] = makeLdRegD8(r);
        }
    }

    // LD A, (rr) and LD (rr), A for BC/DE
    for ([_]U16Register{ .bc, .de }, 0..) |rr, i| {
        table[0x02 + i * 16] = makeLdU16PtrReg(rr, .a);
        table[0x0A + i * 16] = makeLdRegU16Ptr(.a, rr);
    }

    // Jumps, Calls, Returns
    const conds = [_]Conditions{ .nz, .z, .nc, .c };
    for (conds, 0..) |cond, i| {
        table[0x20 + i * 8] = makeJumpRelative(cond);
        table[0xC0 + i * 8] = makeRet(cond);
        table[0xC2 + i * 8] = makeJumpAbsolute(cond);
        table[0xC4 + i * 8] = makeCall(cond);
    }
    table[0x18] = makeJumpRelative(.always);
    table[0xC3] = makeJumpAbsolute(.always);

    // LD r, r'
    for (0..8) |i| {
        const dst_reg = getReg(i);
        for (0..8) |j| {
            const src_reg = getReg(j);
            const opcode = 0x40 + i * 8 + j;
            if (opcode == 0x76) continue;

            if (dst_reg) |dst| {
                if (src_reg) |src| {
                    table[opcode] = makeLdRegReg(dst, src);
                } else {
                    table[opcode] = makeLdRegHlPtr(dst);
                }
            } else {
                if (src_reg) |src| {
                    table[opcode] = makeLdHlPtrReg(src);
                }
            }
        }
    }

    // Arithmetic
    for (0..8) |i| {
        const reg = getReg(i);
        const src_type: SourceType = if (reg == null) .hl_ptr else .register;
        table[0x80 + i] = makeAdd(src_type, reg);
        table[0x88 + i] = makeAdc(src_type, reg);
        table[0x90 + i] = makeSub(src_type, reg);
        table[0x98 + i] = makeSbc(src_type, reg);
        table[0xA0 + i] = makeAnd(src_type, reg);
        table[0xA8 + i] = makeXor(src_type, reg);
        table[0xB0 + i] = makeOr(src_type, reg);
        table[0xB8 + i] = makeCp(src_type, reg);
    }

    // RST
    for (0..8) |i| {
        const opcode = 0xC7 + i * 8;
        table[opcode] = makeRST(opcode);
    }

    break :blk table;
};

const cb_instruction_table = blk: {
    var table = [_]InstructionFn{CPU.unknown_opcode} ** 256;

    const getReg = struct {
        fn f(comptime i: usize) ?Register {
            return switch (i) {
                0 => .b,
                1 => .c,
                2 => .d,
                3 => .e,
                4 => .h,
                5 => .l,
                6 => null,
                7 => .a,
                else => unreachable,
            };
        }
    }.f;

    for (0..8) |i| {
        const reg = getReg(i);
        const src_type: SourceType = if (reg == null) .hl_ptr else .register;

        table[0x00 + i] = if (reg) |r| makeRLC(r) else makeRLC16(.hl);
        table[0x08 + i] = if (reg) |r| makeRRC(r) else makeRRC16(.hl);
        table[0x10 + i] = if (reg) |r| makeRL(r) else makeRL16(.hl);
        table[0x18 + i] = if (reg) |r| makeRR(r) else makeRR16(.hl);
        table[0x20 + i] = if (reg) |r| makeSLA(r) else makeSLA16(.hl);
        table[0x28 + i] = if (reg) |r| makeSRA(r) else makeSRA16(.hl);
        table[0x30 + i] = makeSWAP(src_type, reg);
        table[0x38 + i] = if (reg) |r| makeSRL(r) else makeSRL16(.hl);

        for (0..8) |bit| {
            table[0x40 + bit * 8 + i] = makeBIT(@intCast(bit), src_type, reg);
            table[0x80 + bit * 8 + i] = makeRES(@intCast(bit), src_type, reg);
            table[0xC0 + bit * 8 + i] = makeSET(@intCast(bit), src_type, reg);
        }
    }

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

    ime_state: ImeState = .disabled,
    halted: bool = false,
    haltBug: bool = false,

    memory: MMU,

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
        std.debug.print("\nExecuting PC: 0x{X:0>4}, Opcode: 0x{X:0>2}\n", .{ self.pc, opcode });

        // Implementation of HALT hardware bug
        if (self.haltBug) {
            self.haltBug = false;
        } else {
            self.pc +%= 1;
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
        self.sp = 0xFFFE;
        self.pc = 0x0100;

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

    fn cb_prefix(self: *CPU) void {
        const opcode = self.memory.read(self.pc, u8);
        self.pc += 1;
        cb_instruction_table[opcode](self);
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

    fn ld_a8_a(self: *CPU) void {
        const offset = self.memory.read(self.pc, u8);
        self.pc +%= 1;
        const address = HIGH_RAM_BASE_ADDRESS + offset;
        self.memory.write(address, self.a);
    }

    fn ld_c_a(self: *CPU) void {
        const address: u16 = HIGH_RAM_BASE_ADDRESS + self.c;
        self.memory.write(address, self.a);
    }

    fn ldh_a_a8(self: *CPU) void {
        const offset = self.memory.read(self.pc, u8);
        self.pc +%= 1;
        const address = HIGH_RAM_BASE_ADDRESS + offset;
        self.a = self.memory.read(address, u8);
    }

    fn ldh_a_c(self: *CPU) void {
        const address: u16 = HIGH_RAM_BASE_ADDRESS + self.c;
        self.a = self.memory.read(address, u8);
    }

    fn ld_a16_a(self: *CPU) void {
        const address = self.memory.read(self.pc, u16);
        self.pc +%= 2;
        self.memory.write(address, self.a);
    }

    fn ld_hl_sp_r8(self: *CPU) void {
        const val: i8 = @bitCast(self.memory.read(self.pc, u8));
        self.pc +%= 1;
        const sp: i16 = @bitCast(self.sp);
        const result: u16 = @bitCast(sp +% val);

        self.unsetFlag(.z);
        self.unsetFlag(.n);
        self.checkFlag(.h, ((self.sp & 0x0F) + (@as(u16, @bitCast(@as(i16, val))) & 0x0F)) > 0x0F);
        self.checkFlag(.c, ((self.sp & 0xFF) + (@as(u16, @bitCast(@as(i16, val))) & 0xFF)) > 0xFF);

        self.setU16Register(.hl, result);
    }

    fn ld_sp_hl(self: *CPU) void {
        self.sp = self.getU16Register(.hl);
    }

    fn ld_a_a16(self: *CPU) void {
        const address: u16 = self.memory.read(self.pc, u16);
        self.pc +%= 2;
        self.a = self.memory.read(address, u8);
    }

    fn add_sp_r8(self: *CPU) void {
        const val: i8 = @bitCast(self.memory.read(self.pc, u8));
        self.pc +%= 1;
        const sp: i16 = @bitCast(self.sp);
        const result: u16 = @bitCast(sp +% val);

        self.unsetFlag(.z);
        self.unsetFlag(.n);
        self.checkFlag(.h, ((self.sp & 0x0F) + (@as(u16, @bitCast(@as(i16, val))) & 0x0F)) > 0x0F);
        self.checkFlag(.c, ((self.sp & 0xFF) + (@as(u16, @bitCast(@as(i16, val))) & 0xFF)) > 0xFF);

        self.sp = result;
    }

    fn jp_hl(self: *CPU) void {
        self.pc = self.getU16Register(.hl);
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
        const bit7: u8 = self.a & BIT7;
        self.a <<= 1;
        self.a |= carry_flag;

        self.unsetFlag(.z);
        self.unsetFlag(.n);
        self.unsetFlag(.h);
        self.checkFlag(.c, bit7 != 0);
    }

    fn rra(self: *CPU) void {
        const carry_flag: u8 = (self.f << 3) & BIT7;
        const bit0: u8 = self.a & 1;
        self.a >>= 1;
        self.a |= carry_flag;

        self.unsetFlag(.z);
        self.unsetFlag(.n);
        self.unsetFlag(.h);
        self.checkFlag(.c, bit0 != 0);
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
        self.pc +%= 2;
        self.memory.write(address, self.sp);
    }

    fn ld_sp_d16(self: *CPU) void {
        const d16 = self.memory.read(self.pc, u16);
        self.pc +%= 2;
        self.sp = d16;
    }

    // TODO: Study complex stop flowchart
    fn stop(self: *CPU) void {
        const button_pressed = self.memory.isAnyButtonPressed();
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
        self.pc +%= 1;
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

    fn cpl(self: *CPU) void {
        self.a = ~self.a;
        self.setFlag(.n);
        self.setFlag(.h);
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

    fn checkConditional(self: *CPU, comptime cond: Conditions) bool {
        return switch (cond) {
            .always => true,
            .nz => (self.f & BIT7) != BIT7,
            .z => (self.f & BIT7) == BIT7,
            .nc => (self.f & 0b00010000) != 0b00010000,
            .c => (self.f & 0b00010000) == 0b00010000,
        };
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
