const std = @import("std");
const hw = @import("hw");
const Mmu = @import("mmu").Mmu;
const Mbc = @import("mbc").Mbc;
const interrupts = @import("interrupts");
const Interrupts = interrupts.Interrupts;
const InterruptBit = interrupts.InterruptBit;
const Timer = @import("timer").Timer;
const Sdt = @import("sdt").Sdt;

const InstructionFn = *const fn (*Cpu) void; // Returns the amount of cycles it took to execute
const Flag = enum(u3) { z = 7, n = 6, h = 5, c = 4 };
const Register = enum { a, f, b, c, d, e, h, l };
const U16Register = enum { bc, de, hl, af, sp, pc };
const Conditions = enum { always, nz, z, nc, c }; // Used for conditional instructions like CALL or JP
const ImeState = enum { disabled, enable_pending, enabled }; // interrupts are sometimes delayed one cycle
const SourceType = enum { register, hl_ptr, immediate };

// Helper function to fetch operand value based on source type
fn fetchSource(cpu: *Cpu, comptime src_type: SourceType, comptime reg: ?Register) u8 {
    return switch (src_type) {
        .register => cpu.getRegister(reg.?).*,
        .hl_ptr => cpu.mmu.read(cpu.getU16Register(.hl), u8),
        .immediate => blk: {
            const val = cpu.mmu.read(cpu.pc, u8);
            cpu.pc += 1;
            break :blk val;
        },
    };
}

// Comptime instruction generators
fn makeLdRegReg(comptime dst: Register, comptime src: Register) InstructionFn {
    return struct {
        fn f(cpu: *Cpu) void {
            cpu.getRegister(dst).* = cpu.getRegister(src).*;
        }
    }.f;
}

fn makeLdRegHlPtr(comptime dst: Register) InstructionFn {
    return struct {
        fn f(cpu: *Cpu) void {
            cpu.getRegister(dst).* = cpu.mmu.read(cpu.getU16Register(.hl), u8);
        }
    }.f;
}

fn makeLdRegU16Ptr(comptime dst: Register, comptime src: U16Register) InstructionFn {
    return struct {
        fn f(cpu: *Cpu) void {
            const address = cpu.getU16Register(src);
            cpu.getRegister(dst).* = cpu.mmu.read(address, u8);
        }
    }.f;
}

fn makeLdU16PtrReg(comptime dst: U16Register, comptime src: Register) InstructionFn {
    return struct {
        fn f(cpu: *Cpu) void {
            const address = cpu.getU16Register(dst);
            cpu.mmu.write(address, cpu.getRegister(src).*);
        }
    }.f;
}

fn makeLdRegD8(comptime dst: Register) InstructionFn {
    return struct {
        fn f(cpu: *Cpu) void {
            cpu.getRegister(dst).* = cpu.mmu.read(cpu.pc, u8);
            cpu.pc += 1;
        }
    }.f;
}

fn makeLdU16RegD16(comptime dst: U16Register) InstructionFn {
    return struct {
        fn f(cpu: *Cpu) void {
            const d16 = cpu.mmu.read(cpu.pc, u16);
            cpu.pc += 2;
            cpu.setU16Register(dst, d16);
        }
    }.f;
}

fn makeLdHlPtrReg(comptime src: Register) InstructionFn {
    return struct {
        fn f(cpu: *Cpu) void {
            cpu.mmu.write(cpu.getU16Register(.hl), cpu.getRegister(src).*);
        }
    }.f;
}

fn makeAdd(comptime src_type: SourceType, comptime reg: ?Register) InstructionFn {
    return struct {
        fn f(cpu: *Cpu) void {
            const a = cpu.a;
            const value = fetchSource(cpu, src_type, reg);
            cpu.a = a +% value;
            cpu.checkFlag(.z, cpu.a == 0);
            cpu.unsetFlag(.n);
            cpu.checkFlag(.h, (a & 0x0F) + (value & 0x0F) > 0x0F);
            cpu.checkFlag(.c, @as(u16, a) + value > 0xFF);
        }
    }.f;
}

fn makeSub(comptime src_type: SourceType, comptime reg: ?Register) InstructionFn {
    return struct {
        fn f(cpu: *Cpu) void {
            const a = cpu.a;
            const value = fetchSource(cpu, src_type, reg);
            cpu.a = a -% value;
            cpu.checkFlag(.z, cpu.a == 0);
            cpu.setFlag(.n);
            cpu.checkFlag(.h, (a & 0x0F) < (value & 0x0F));
            cpu.checkFlag(.c, a < value);
        }
    }.f;
}

fn makeAnd(comptime src_type: SourceType, comptime reg: ?Register) InstructionFn {
    return struct {
        fn f(cpu: *Cpu) void {
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
        fn f(cpu: *Cpu) void {
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
        fn f(cpu: *Cpu) void {
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
        fn f(cpu: *Cpu) void {
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
        fn f(cpu: *Cpu) void {
            const reg = cpu.getRegister(dst);
            const old = reg.*;
            reg.* +%= 1;
            cpu.updateIncFlags(old, reg.*, false);
        }
    }.f;
}

fn makeDecReg(comptime dst: Register) InstructionFn {
    return struct {
        fn f(cpu: *Cpu) void {
            const reg = cpu.getRegister(dst);
            const old = reg.*;
            reg.* -%= 1;
            cpu.updateIncFlags(old, reg.*, true);
        }
    }.f;
}

fn makeIncU16Reg(comptime dst: U16Register) InstructionFn {
    return struct {
        fn f(cpu: *Cpu) void {
            cpu.timer.tick(); // 16 bit math takes an extra M-cycle
            cpu.setU16Register(dst, cpu.getU16Register(dst) +% 1);
        }
    }.f;
}

fn makeDecU16Reg(comptime dst: U16Register) InstructionFn {
    return struct {
        fn f(cpu: *Cpu) void {
            cpu.timer.tick();
            cpu.setU16Register(dst, cpu.getU16Register(dst) -% 1);
        }
    }.f;
}

fn makeAddHl(comptime src: U16Register) InstructionFn {
    return struct {
        fn f(cpu: *Cpu) void {
            cpu.timer.tick();
            const hl = cpu.getU16Register(.hl);
            const value = cpu.getU16Register(src);
            const result = hl +% value;
            cpu.setU16Register(.hl, result);
            cpu.unsetFlag(.n);
            cpu.checkFlag(.h, (hl & 0x0FFF) + (value & 0x0FFF) > 0x0FFF);
            cpu.checkFlag(.c, @as(u32, hl) + value > 0xFFFF);
        }
    }.f;
}

fn makeAdc(comptime src_type: SourceType, comptime reg: ?Register) InstructionFn {
    return struct {
        fn f(cpu: *Cpu) void {
            const a = cpu.a;
            const carry: u8 = @intFromBool(cpu.readFlag(.c));
            const value = fetchSource(cpu, src_type, reg);
            const result: u16 = @as(u16, a) +% value +% carry;
            cpu.a = @truncate(result);

            cpu.checkFlag(.z, cpu.a == 0);
            cpu.unsetFlag(.n);
            cpu.checkFlag(.h, (a & 0xF) + (value & 0xF) + carry > 0xF);
            cpu.checkFlag(.c, result > 0xFF);
        }
    }.f;
}

fn makeSbc(comptime src_type: SourceType, comptime reg: ?Register) InstructionFn {
    return struct {
        fn f(cpu: *Cpu) void {
            const a = cpu.a;
            const carry: u8 = @intFromBool(cpu.readFlag(.c));
            const value = fetchSource(cpu, src_type, reg);
            const result: u16 = @as(u16, a) -% value -% carry;
            cpu.a = @truncate(result);

            cpu.checkFlag(.z, cpu.a == 0);
            cpu.setFlag(.n);
            cpu.checkFlag(.h, (a & 0x0F) < @as(u16, value & 0x0F) + carry);
            cpu.checkFlag(.c, result > 0xFF);
        }
    }.f;
}

fn makeJumpRelative(comptime cond: Conditions) InstructionFn {
    return struct {
        fn f(cpu: *Cpu) void {
            const offset: i8 = @bitCast(cpu.mmu.read(cpu.pc, u8));
            cpu.pc += 1;

            if (cpu.checkConditional(cond)) {
                cpu.timer.tick(); // Extra 4 cycles if condition is true
                const offset_i16: i16 = offset;
                const offset_u16: u16 = @bitCast(offset_i16);
                cpu.pc +%= offset_u16;
            }
        }
    }.f;
}

fn makeJumpAbsolute(comptime cond: Conditions) InstructionFn {
    return struct {
        fn f(cpu: *Cpu) void {
            const address: u16 = cpu.mmu.read(cpu.pc, u16);
            cpu.pc +%= 2;

            if (cpu.checkConditional(cond)) {
                cpu.timer.tick();
                cpu.pc = address;
            }
        }
    }.f;
}

fn makeCall(comptime cond: Conditions) InstructionFn {
    return struct {
        fn f(cpu: *Cpu) void {
            const address: u16 = cpu.mmu.read(cpu.pc, u16);
            cpu.pc +%= 2;

            if (cpu.checkConditional(cond)) {
                // pushU16 automatically ticks timer thrice
                cpu.pushU16(cpu.pc);
                cpu.pc = address;
            }
        }
    }.f;
}

fn makeRet(comptime cond: Conditions) InstructionFn {
    return struct {
        fn f(cpu: *Cpu) void {
            if (cond != .always) {
                cpu.timer.tick(); // Extra tick for conditionals
            }

            if (cpu.checkConditional(cond)) {
                cpu.pc = cpu.popU16();
                cpu.timer.tick();
            }
        }
    }.f;
}

fn makePush(comptime pair: U16Register) InstructionFn {
    return struct {
        fn f(cpu: *Cpu) void {
            const value = cpu.getU16Register(pair);
            cpu.pushU16(value);
        }
    }.f;
}

fn makePop(comptime pair: U16Register) InstructionFn {
    return struct {
        fn f(cpu: *Cpu) void {
            const value = cpu.popU16();
            cpu.setU16Register(pair, value);
        }
    }.f;
}

fn makeRST(comptime opcode: u8) InstructionFn {
    return struct {
        fn f(cpu: *Cpu) void {
            cpu.pushU16(cpu.pc);
            cpu.pc = opcode & 0b00111000; // black magic
        }
    }.f;
}

fn makeRLC(comptime src_type: SourceType, comptime reg_name: ?Register) InstructionFn {
    return struct {
        fn f(cpu: *Cpu) void {
            const val = fetchSource(cpu, src_type, reg_name);
            const bit7 = (val >> 7) & 1;
            const result = (val << 1) | bit7;

            switch (src_type) {
                .register => cpu.getRegister(reg_name.?).* = result,
                .hl_ptr => cpu.mmu.write(cpu.getU16Register(.hl), result),
                .immediate => @compileError("makeRLC does not support .immediate source type"),
            }

            cpu.checkFlag(.z, result == 0);
            cpu.checkFlag(.c, bit7 != 0);
            cpu.unsetFlag(.h);
            cpu.unsetFlag(.n);
        }
    }.f;
}

fn makeRRC(comptime src_type: SourceType, comptime reg_name: ?Register) InstructionFn {
    return struct {
        fn f(cpu: *Cpu) void {
            const val = fetchSource(cpu, src_type, reg_name);
            const bit0 = val & 1;
            const result = (val >> 1) | (bit0 << 7);

            switch (src_type) {
                .register => cpu.getRegister(reg_name.?).* = result,
                .hl_ptr => cpu.mmu.write(cpu.getU16Register(.hl), result),
                .immediate => @compileError("makeRRC does not support .immediate source type"),
            }

            cpu.checkFlag(.z, result == 0);
            cpu.checkFlag(.c, bit0 != 0);
            cpu.unsetFlag(.h);
            cpu.unsetFlag(.n);
        }
    }.f;
}

fn makeRR(comptime src_type: SourceType, comptime reg_name: ?Register) InstructionFn {
    return struct {
        fn f(cpu: *Cpu) void {
            const val = fetchSource(cpu, src_type, reg_name);
            const bit0 = val & 1;
            const c_flag = cpu.readFlag(.c);
            const result = (val >> 1) | (@as(u8, @intFromBool(c_flag)) << 7);

            switch (src_type) {
                .register => cpu.getRegister(reg_name.?).* = result,
                .hl_ptr => cpu.mmu.write(cpu.getU16Register(.hl), result),
                .immediate => unreachable,
            }

            cpu.checkFlag(.c, bit0 != 0);
            cpu.checkFlag(.z, result == 0);
            cpu.unsetFlag(.h);
            cpu.unsetFlag(.n);
        }
    }.f;
}

fn makeRL(comptime src_type: SourceType, comptime reg_name: ?Register) InstructionFn {
    return struct {
        fn f(cpu: *Cpu) void {
            const val = fetchSource(cpu, src_type, reg_name);
            const bit7 = (val >> 7) & 1;
            const c_flag = cpu.readFlag(.c);
            const result = (val << 1) | @intFromBool(c_flag);

            switch (src_type) {
                .register => cpu.getRegister(reg_name.?).* = result,
                .hl_ptr => cpu.mmu.write(cpu.getU16Register(.hl), result),
                .immediate => unreachable,
            }

            cpu.checkFlag(.c, bit7 != 0);
            cpu.checkFlag(.z, result == 0);
            cpu.unsetFlag(.h);
            cpu.unsetFlag(.n);
        }
    }.f;
}

fn makeSLA(comptime src_type: SourceType, comptime reg_name: ?Register) InstructionFn {
    return struct {
        fn f(cpu: *Cpu) void {
            const val = fetchSource(cpu, src_type, reg_name);
            const bit7 = (val >> 7) & 1;
            const result = val << 1;

            switch (src_type) {
                .register => cpu.getRegister(reg_name.?).* = result,
                .hl_ptr => cpu.mmu.write(cpu.getU16Register(.hl), result),
                .immediate => unreachable,
            }

            cpu.checkFlag(.c, bit7 != 0);
            cpu.checkFlag(.z, result == 0);
            cpu.unsetFlag(.h);
            cpu.unsetFlag(.n);
        }
    }.f;
}

fn makeSRA(comptime src_type: SourceType, comptime reg_name: ?Register) InstructionFn {
    return struct {
        fn f(cpu: *Cpu) void {
            const val = fetchSource(cpu, src_type, reg_name);
            const bit0 = val & 1;
            const bit7 = val & 0x80;
            const result = (val >> 1) | bit7;

            switch (src_type) {
                .register => cpu.getRegister(reg_name.?).* = result,
                .hl_ptr => cpu.mmu.write(cpu.getU16Register(.hl), result),
                .immediate => unreachable,
            }

            cpu.checkFlag(.c, bit0 != 0);
            cpu.checkFlag(.z, result == 0);
            cpu.unsetFlag(.h);
            cpu.unsetFlag(.n);
        }
    }.f;
}

fn makeSRL(comptime src_type: SourceType, comptime reg_name: ?Register) InstructionFn {
    return struct {
        fn f(cpu: *Cpu) void {
            const val = fetchSource(cpu, src_type, reg_name);
            const bit0 = val & 1;
            const result = val >> 1;

            switch (src_type) {
                .register => cpu.getRegister(reg_name.?).* = result,
                .hl_ptr => cpu.mmu.write(cpu.getU16Register(.hl), result),
                .immediate => unreachable,
            }

            cpu.checkFlag(.c, bit0 != 0);
            cpu.checkFlag(.z, result == 0);
            cpu.unsetFlag(.h);
            cpu.unsetFlag(.n);
        }
    }.f;
}

fn makeBIT(comptime bit_num: u3, comptime src_type: SourceType, comptime reg: ?Register) InstructionFn {
    return struct {
        fn f(cpu: *Cpu) void {
            const dst = fetchSource(cpu, src_type, reg);
            cpu.checkFlag(.z, (dst & (@as(u8, 1) << bit_num)) == 0);
            cpu.unsetFlag(.n);
            cpu.setFlag(.h);
        }
    }.f;
}

fn makeRES(comptime bit_num: u3, comptime src_type: SourceType, comptime reg: ?Register) InstructionFn {
    return struct {
        fn f(cpu: *Cpu) void {
            switch (src_type) {
                .register => {
                    cpu.getRegister(reg.?).* &= ~(@as(u8, 1) << bit_num);
                },
                .hl_ptr => {
                    const address = cpu.getU16Register(.hl);
                    var val = cpu.mmu.read(address, u8);
                    val &= ~(@as(u8, 1) << bit_num);
                    cpu.mmu.write(address, val);
                },
                .immediate => @compileError("Immediate SourceType is not supported on RES instructions."),
            }
        }
    }.f;
}

fn makeSET(comptime bit_num: u3, comptime src_type: SourceType, comptime reg: ?Register) InstructionFn {
    return struct {
        fn f(cpu: *Cpu) void {
            switch (src_type) {
                .hl_ptr => {
                    const address = cpu.getU16Register(.hl);
                    var val = cpu.mmu.read(address, u8);
                    val |= (@as(u8, 1) << bit_num);
                    cpu.mmu.write(address, val);
                },
                .register => cpu.getRegister(reg.?).* |= (@as(u8, 1) << bit_num),
                .immediate => @compileError("Immediate SourceType is not supported on SET instructions."),
            }
        }
    }.f;
}

fn makeSWAP(comptime src: SourceType, comptime reg_name: ?Register) InstructionFn {
    return struct {
        fn f(cpu: *Cpu) void {
            switch (src) {
                .register => {
                    const reg = cpu.getRegister(reg_name.?);
                    const high = reg.* >> 4;
                    reg.* = (reg.* << 4) | high;
                    cpu.checkFlag(.z, reg.* == 0);
                },
                .hl_ptr => {
                    const address = cpu.getU16Register(.hl);
                    const val = cpu.mmu.read(address, u8);
                    const high = val >> 4;
                    const result = (val << 4) | high;
                    cpu.mmu.write(address, result);
                    cpu.checkFlag(.z, result == 0);
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
    var table = [_]InstructionFn{Cpu.unknown_opcode} ** 256;

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

    table[0x00] = Cpu.nop;
    table[0x07] = Cpu.rlca;
    table[0x08] = Cpu.ld_a16_ptr_sp;
    table[0x0F] = Cpu.rrca;
    table[0x10] = Cpu.stop;
    table[0x17] = Cpu.rla;
    table[0x1F] = Cpu.rra;
    table[0x22] = Cpu.ld_hl_ptr_plus_a;
    table[0x27] = Cpu.daa;
    table[0x2A] = Cpu.ld_a_hl_ptr_plus;
    table[0x2F] = Cpu.cpl;
    table[0x31] = Cpu.ld_sp_d16;
    table[0x32] = Cpu.ld_hl_ptr_minus_a;
    table[0x34] = Cpu.inc_hl_ptr;
    table[0x35] = Cpu.dec_hl_ptr;
    table[0x36] = Cpu.ld_hl_ptr_d8;
    table[0x37] = Cpu.scf;
    table[0x3A] = Cpu.ld_a_hl_ptr_minus;
    table[0x3F] = Cpu.ccf;
    table[0x76] = Cpu.halt;
    table[0xC1] = makePop(.bc);
    table[0xC5] = makePush(.bc);
    table[0xC6] = makeAdd(.immediate, null);
    table[0xC9] = makeRet(.always);
    table[0xCB] = Cpu.cb_prefix;
    table[0xCD] = makeCall(.always);
    table[0xCE] = makeAdc(.immediate, null);
    table[0xD1] = makePop(.de);
    table[0xD5] = makePush(.de);
    table[0xD6] = makeSub(.immediate, null);
    table[0xD9] = Cpu.reti;
    table[0xDE] = makeSbc(.immediate, null);
    table[0xE0] = Cpu.ld_a8_a;
    table[0xE1] = makePop(.hl);
    table[0xE2] = Cpu.ld_c_a;
    table[0xE5] = makePush(.hl);
    table[0xE6] = makeAnd(.immediate, null);
    table[0xE8] = Cpu.add_sp_r8;
    table[0xE9] = Cpu.jp_hl;
    table[0xEA] = Cpu.ld_a16_a;
    table[0xEE] = makeXor(.immediate, null);
    table[0xF0] = Cpu.ldh_a_a8;
    table[0xF1] = makePop(.af);
    table[0xF2] = Cpu.ldh_a_c;
    table[0xF3] = Cpu.di;
    table[0xF5] = makePush(.af);
    table[0xF6] = makeOr(.immediate, null);
    table[0xF8] = Cpu.ld_hl_sp_r8;
    table[0xF9] = Cpu.ld_sp_hl;
    table[0xFA] = Cpu.ld_a_a16;
    table[0xFB] = Cpu.ei;
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
    var table = [_]InstructionFn{Cpu.unknown_opcode} ** 256;

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

        table[0x00 + i] = makeRLC(src_type, reg);
        table[0x08 + i] = makeRRC(src_type, reg);
        table[0x10 + i] = makeRL(src_type, reg);
        table[0x18 + i] = makeRR(src_type, reg);
        table[0x20 + i] = makeSLA(src_type, reg);
        table[0x28 + i] = makeSRA(src_type, reg);
        table[0x30 + i] = makeSWAP(src_type, reg);
        table[0x38 + i] = makeSRL(src_type, reg);

        for (0..8) |bit| {
            table[0x40 + bit * 8 + i] = makeBIT(@intCast(bit), src_type, reg);
            table[0x80 + bit * 8 + i] = makeRES(@intCast(bit), src_type, reg);
            table[0xC0 + bit * 8 + i] = makeSET(@intCast(bit), src_type, reg);
        }
    }

    break :blk table;
};

pub const Cpu = struct {
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
    halt_bug: bool = false,

    mmu: *Mmu,
    interrupts: *Interrupts,
    timer: *Timer,
    sdt: *Sdt,

    pub fn step(self: *Cpu) void {
        if (self.interrupts.getPending()) |interrupt| {
            self.halted = false;
            if (self.ime_state == .enabled) {
                self.handleInterrupt(interrupt);
                return;
            }
        }

        if (self.ime_state == .enable_pending) self.ime_state = .enabled;

        if (self.halted) {
            self.timer.tick();
            return;
        }

        const opcode = self.mmu.read(self.pc, u8);

        // Implementation of HALT hardware bug
        if (self.halt_bug) {
            self.halt_bug = false;
        } else {
            self.pc +%= 1;
        }

        instruction_table[opcode](self);
    }

    // Fast boot implementation. Initializes memory to the post boot state and does nothing more.
    pub fn boot(self: *Cpu) void {
        self.a = 0x01;
        self.f = 0xB0;
        self.c = 0x13;
        self.e = 0xD8;
        self.h = 0x01;
        self.l = 0x4D;
        self.sp = hw.Boot.sp;
        self.pc = hw.Boot.pc;

        self.mmu.bootWrite(hw.Io.tima, 0x00);
        self.mmu.bootWrite(hw.Io.tma, 0x00);
        self.mmu.bootWrite(hw.Io.tac, 0x00);
        self.mmu.bootWrite(hw.Io.nr10, 0x80);
        self.mmu.bootWrite(hw.Io.nr11, 0xBF);
        self.mmu.bootWrite(hw.Io.nr12, 0xF3);
        self.mmu.bootWrite(hw.Io.nr14, 0xBF);
        self.mmu.bootWrite(hw.Io.nr21, 0x3F);
        self.mmu.bootWrite(hw.Io.nr22, 0x00);
        self.mmu.bootWrite(hw.Io.nr24, 0xBF);
        self.mmu.bootWrite(hw.Io.nr30, 0x7F);
        self.mmu.bootWrite(hw.Io.nr31, 0xFF);
        self.mmu.bootWrite(hw.Io.nr32, 0x9F);
        self.mmu.bootWrite(hw.Io.nr34, 0xBF);
        self.mmu.bootWrite(hw.Io.nr41, 0xFF);
        self.mmu.bootWrite(hw.Io.nr42, 0x00);
        self.mmu.bootWrite(hw.Io.nr43, 0x00);
        self.mmu.bootWrite(hw.Io.nr44, 0xBF);
        self.mmu.bootWrite(hw.Io.nr50, 0x77);
        self.mmu.bootWrite(hw.Io.nr51, 0xF3);
        self.mmu.bootWrite(hw.Io.nr52, 0xF1);
        self.mmu.bootWrite(hw.Io.lcdc, 0x91);
        self.mmu.bootWrite(hw.Io.scy, 0x00);
        self.mmu.bootWrite(hw.Io.scx, 0x00);
        self.mmu.bootWrite(hw.Io.lyc, 0x00);
        self.mmu.bootWrite(hw.Io.bgp, 0xFC);
        self.mmu.bootWrite(hw.Io.obp0, 0xFF);
        self.mmu.bootWrite(hw.Io.obp1, 0xFF);
        self.mmu.bootWrite(hw.Io.wy, 0x00);
        self.mmu.bootWrite(hw.Io.wx, 0x00);
        self.mmu.bootWrite(hw.Map.ie_reg, 0x00);
    }

    fn handleInterrupt(self: *Cpu, interrupt: InterruptBit) void {
        // std.debug.print("[INT] Servicing {s} from PC: 0x{X:0>4}\n", .{@tagName(interrupt), self.pc});
        self.ime_state = .disabled;
        self.halted = false;
        self.interrupts.acknowledge(interrupt);

        self.timer.tick();
        self.pushU16(self.pc);

        self.pc = switch (interrupt) {
            .vblank => hw.Interrupt.vblank,
            .lcd => hw.Interrupt.lcd,
            .timer => hw.Interrupt.timer,
            .serial => hw.Interrupt.serial,
            .joypad => hw.Interrupt.joypad,
        };

        self.timer.tick();
    }

    // Individual instruction implementations
    fn nop(self: *Cpu) void {
        _ = self;
    }

    fn cb_prefix(self: *Cpu) void {
        const opcode = self.mmu.read(self.pc, u8);
        self.pc += 1;
        cb_instruction_table[opcode](self);
    }

    fn ld_hl_ptr_plus_a(self: *Cpu) void {
        const address = self.getU16Register(.hl);
        self.mmu.write(address, self.a);
        self.setU16Register(.hl, self.getU16Register(.hl) + 1);
    }

    fn ld_a_hl_ptr_plus(self: *Cpu) void {
        const address = self.getU16Register(.hl);
        self.a = self.mmu.read(address, u8);
        self.setU16Register(.hl, self.getU16Register(.hl) + 1);
    }

    fn ld_hl_ptr_minus_a(self: *Cpu) void {
        const address = self.getU16Register(.hl);
        self.mmu.write(address, self.a);
        self.setU16Register(.hl, self.getU16Register(.hl) - 1);
    }

    fn ld_a_hl_ptr_minus(self: *Cpu) void {
        const address = self.getU16Register(.hl);
        self.a = self.mmu.read(address, u8);
        self.setU16Register(.hl, self.getU16Register(.hl) - 1);
    }

    fn ld_a8_a(self: *Cpu) void {
        const offset = self.mmu.read(self.pc, u8);
        self.pc +%= 1;
        const address = hw.Map.io.start + offset;
        self.mmu.write(address, self.a);
    }

    fn ld_c_a(self: *Cpu) void {
        const address: u16 = hw.Map.io.start + self.c;
        self.mmu.write(address, self.a);
    }

    fn ldh_a_a8(self: *Cpu) void {
        const offset = self.mmu.read(self.pc, u8);
        self.pc +%= 1;
        const address = hw.Map.io.start + offset;
        self.a = self.mmu.read(address, u8);
    }

    fn ldh_a_c(self: *Cpu) void {
        const address: u16 = hw.Map.io.start + self.c;
        self.a = self.mmu.read(address, u8);
    }

    fn ld_a16_a(self: *Cpu) void {
        const address = self.mmu.read(self.pc, u16);
        self.pc +%= 2;
        self.mmu.write(address, self.a);
    }

    fn ld_hl_sp_r8(self: *Cpu) void {
        const val: i8 = @bitCast(self.mmu.read(self.pc, u8));

        self.timer.tick();
        self.pc +%= 1;
        const sp: i16 = @bitCast(self.sp);
        const result: u16 = @bitCast(sp +% val);

        self.unsetFlag(.z);
        self.unsetFlag(.n);
        self.checkFlag(.h, ((self.sp & 0x0F) + (@as(u16, @bitCast(@as(i16, val))) & 0x0F)) > 0x0F);
        self.checkFlag(.c, ((self.sp & 0xFF) + (@as(u16, @bitCast(@as(i16, val))) & 0xFF)) > 0xFF);

        self.setU16Register(.hl, result);
    }

    fn ld_sp_hl(self: *Cpu) void {
        self.timer.tick();
        self.sp = self.getU16Register(.hl);
    }

    fn ld_a_a16(self: *Cpu) void {
        const address: u16 = self.mmu.read(self.pc, u16);
        self.pc +%= 2;
        self.a = self.mmu.read(address, u8);
    }

    fn add_sp_r8(self: *Cpu) void {
        const val: i8 = @bitCast(self.mmu.read(self.pc, u8));
        self.timer.tick();
        self.pc +%= 1;
        const sp: i16 = @bitCast(self.sp);
        const result: u16 = @bitCast(sp +% val);

        self.unsetFlag(.z);
        self.unsetFlag(.n);
        self.checkFlag(.h, ((self.sp & 0x0F) + (@as(u16, @bitCast(@as(i16, val))) & 0x0F)) > 0x0F);
        self.checkFlag(.c, ((self.sp & 0xFF) + (@as(u16, @bitCast(@as(i16, val))) & 0xFF)) > 0xFF);

        self.timer.tick();
        self.sp = result;
    }

    fn jp_hl(self: *Cpu) void {
        self.pc = self.getU16Register(.hl);
    }

    fn rlca(self: *Cpu) void {
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

    fn rla(self: *Cpu) void {
        const carry_flag: u8 = (self.f >> 4) & 1;
        const bit7: u8 = self.a & 0b10000000;
        self.a <<= 1;
        self.a |= carry_flag;

        self.unsetFlag(.z);
        self.unsetFlag(.n);
        self.unsetFlag(.h);
        self.checkFlag(.c, bit7 != 0);
    }

    fn rra(self: *Cpu) void {
        const carry_flag: u8 = (self.f << 3) & 0b10000000;
        const bit0: u8 = self.a & 1;
        self.a >>= 1;
        self.a |= carry_flag;

        self.unsetFlag(.z);
        self.unsetFlag(.n);
        self.unsetFlag(.h);
        self.checkFlag(.c, bit0 != 0);
    }

    fn rrca(self: *Cpu) void {
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

    fn ld_a16_ptr_sp(self: *Cpu) void {
        const address: u16 = self.mmu.read(self.pc, u16);
        self.pc +%= 2;
        self.mmu.write(address, self.sp);
    }

    fn ld_sp_d16(self: *Cpu) void {
        const d16 = self.mmu.read(self.pc, u16);
        self.pc +%= 2;
        self.sp = d16;
    }

    // TODO: Study complex stop flowchart
    fn stop(self: *Cpu) void {
        const button_pressed = self.mmu.isAnyButtonPressed();
        if (button_pressed) {
            if (self.interrupts.getPending()) |pending| {
                _ = pending;
                // STOP is a 1 byte opcode, mode doesn't change, DIV doesn't reset.
            } else {
                // STOP is a 2-byte opcode, HALT mode is entered, DIV is not reset
            }
        }
    }

    fn inc_hl_ptr(self: *Cpu) void {
        const address = self.getU16Register(.hl);
        const old = self.mmu.read(address, u8);
        const new = old +% 1;
        self.mmu.write(address, new);
        self.updateIncFlags(old, new, false);
    }

    fn dec_hl_ptr(self: *Cpu) void {
        const address = self.getU16Register(.hl);
        const old = self.mmu.read(address, u8);
        const new = old -% 1;
        self.mmu.write(address, new);
        self.updateIncFlags(old, new, true);
    }

    fn halt(self: *Cpu) void {
        self.halted = true;
        const pending = self.interrupts.getPending();
        if (self.ime_state == .disabled and pending != null) {
            self.halt_bug = true;
        }
    }

    fn ld_hl_ptr_d8(self: *Cpu) void {
        const value = self.mmu.read(self.pc, u8);
        self.pc +%= 1;
        self.mmu.write(self.getU16Register(.hl), value);
    }

    fn daa(self: *Cpu) void {
        var a = self.a;
        var correction: u8 = 0;
        var carry = self.readFlag(.c);

        if (self.readFlag(.h) or (!self.readFlag(.n) and (a & 0x0F) > 0x09)) {
            correction |= 0x06;
        }

        if (self.readFlag(.c) or (!self.readFlag(.n) and a > 0x99)) {
            correction |= 0x60;
            carry = true;
        }

        if (self.readFlag(.n)) {
            a -%= correction;
        } else {
            a +%= correction;
        }

        self.a = a;
        self.checkFlag(.z, a == 0);
        self.unsetFlag(.h);
        self.checkFlag(.c, carry);
    }

    fn scf(self: *Cpu) void {
        self.setFlag(.c);
        self.unsetFlag(.n);
        self.unsetFlag(.h);
    }

    fn ccf(self: *Cpu) void {
        const c_flag = self.readFlag(.c);
        self.checkFlag(.c, !c_flag);
        self.unsetFlag(.n);
        self.unsetFlag(.h);
    }

    fn cpl(self: *Cpu) void {
        self.a = ~self.a;
        self.setFlag(.n);
        self.setFlag(.h);
    }

    fn ei(self: *Cpu) void {
        self.ime_state = .enable_pending;
    }

    fn di(self: *Cpu) void {
        self.ime_state = .disabled;
    }

    fn reti(self: *Cpu) void {
        self.timer.tick();
        self.pc = self.popU16();
        self.ime_state = .enabled; // RETI enables immediately, no delay
    }

    fn unknown_opcode(self: *Cpu) void {
        _ = self;
        @panic("Unknown opcode");
    }

    // Helper functions
    fn getRegister(self: *Cpu, comptime reg: Register) *u8 {
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

    fn getU16Register(self: *Cpu, comptime pair: U16Register) u16 {
        return switch (pair) {
            .bc => (@as(u16, self.b) << 8) | self.c,
            .de => (@as(u16, self.d) << 8) | self.e,
            .hl => (@as(u16, self.h) << 8) | self.l,
            .af => (@as(u16, self.a) << 8) | self.f,
            .sp => self.sp,
            .pc => self.pc,
        };
    }

    fn setU16Register(self: *Cpu, comptime pair: U16Register, val: u16) void {
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
                self.f = @as(u8, @truncate(val)) & 0xF0;
            },
            .sp => self.sp = val,
            .pc => self.pc = val,
        }
    }

    fn setFlag(self: *Cpu, comptime flag: Flag) void {
        self.f |= (@as(u8, 1) << @intFromEnum(flag));
    }

    fn unsetFlag(self: *Cpu, comptime flag: Flag) void {
        self.f &= ~(@as(u8, 1) << @intFromEnum(flag));
    }

    pub fn readFlag(self: *Cpu, comptime flag: Flag) bool {
        return ((self.f >> @intFromEnum(flag)) & 1) == 1;
    }

    fn checkFlag(self: *Cpu, comptime flag: Flag, condition: bool) void {
        if (condition) {
            self.setFlag(flag);
        } else {
            self.unsetFlag(flag);
        }
    }

    fn checkConditional(self: *Cpu, comptime cond: Conditions) bool {
        return switch (cond) {
            .always => true,
            .nz => (self.f & 0b10000000) != 0b10000000,
            .z => (self.f & 0b10000000) == 0b10000000,
            .nc => (self.f & 0b00010000) != 0b00010000,
            .c => (self.f & 0b00010000) == 0b00010000,
        };
    }

    fn updateIncFlags(self: *Cpu, old: u8, new: u8, comptime is_dec: bool) void {
        self.checkFlag(.z, new == 0);
        self.checkFlag(.n, is_dec);
        if (is_dec) {
            self.checkFlag(.h, (old & 0x0F) == 0);
        } else {
            self.checkFlag(.h, (old & 0x0F) + 1 > 0x0F);
        }
    }

    fn pushU16(self: *Cpu, value: u16) void {
        self.timer.tick(); // Incrementing SP takes an extra tick on push only
        self.sp -%= 2;
        self.mmu.write(self.sp, value);
    }

    fn popU16(self: *Cpu) u16 {
        const value = self.mmu.read(self.sp, u16);
        self.sp +%= 2;
        return value;
    }
};
