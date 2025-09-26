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

        switch (opcode) {
            0x00 => { // Nop
            },
            0x01 => { // LD BC,d16
                self.c = self.memory[self.pc];
                self.pc += 1;
                self.b = self.memory[self.pc];
                self.pc += 1;
            },
            0x02 => { // LD (BC), A
                const address = self.getBC();
                self.memory[address] = self.a;
            },
            0x06 => { // LD B, n
                self.b = self.memory[self.pc];
                self.pc += 1;
            },
            0x0A => { // LD A, (BC)
                const address = self.getBC();
                self.a = self.memory[address];
            },
            0x0E => { // LD C,d8
                const n = self.memory[self.pc];
                self.pc += 1;
                self.c = n;
            },
            0x12 => { // LD (DE),A
                const address = self.getDE();
                self.memory[address] = self.a;
            },
            0x16 => { // LD D,d8
                self.d = self.memory[self.pc];
                self.pc += 1;
            },
            else => {
                @panic("Unknown opcode");
            },
        }
    }

    // Helper functions so I don't typo these bit operations lmao
    fn getBC(self: *CPU) u16 {
        return (@as(u16, self.b) << 8) | self.c;
    }

    fn getDE(self: *CPU) u16 {
        return (@as(u16, self.d) << 8) | self.e;
    }
};
