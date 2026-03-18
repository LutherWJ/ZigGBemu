pub const Timings = struct {
    pub const cpuFrequency: comptime_int = 2 >> 22; // 4.194304 MHz
    pub const divRegFrequency: comptime_int = 2 >> 14; // 16384 Hz
    pub const tCyclesPerMCycle: comptime_int = 4;
};

pub const Map = struct {
    pub const rom0 = struct {
        pub const start: u16 = 0x0000;
        pub const end: u16 = 0x3FFF;
        pub const size: usize = end - start + 1;
    };
    pub const romx = struct {
        pub const start: u16 = 0x4000;
        pub const end: u16 = 0x7FFF;
        pub const size: usize = end - start + 1;
    };
    pub const vram = struct {
        pub const start: u16 = 0x8000;
        pub const end: u16 = 0x9FFF;
        pub const size: usize = end - start + 1;
    };
    pub const ext_ram = struct {
        pub const start: u16 = 0xA000;
        pub const end: u16 = 0xBFFF;
        pub const size: usize = end - start + 1;
    };
    pub const wram0 = struct {
        pub const start: u16 = 0xC000;
        pub const end: u16 = 0xCFFF;
        pub const size: usize = end - start + 1;
    };
    pub const wramx = struct {
        pub const start: u16 = 0xD000;
        pub const end: u16 = 0xDFFF;
        pub const size: usize = end - start + 1;
    };
    pub const echo = struct {
        pub const start: u16 = 0xE000;
        pub const end: u16 = 0xFDFF;
        pub const size: usize = end - start + 1;
    };
    pub const oam = struct {
        pub const start: u16 = 0xFE00;
        pub const end: u16 = 0xFE9F;
        pub const size: usize = end - start + 1;
    };
    pub const unusable = struct {
        pub const start: u16 = 0xFEA0;
        pub const end: u16 = 0xFEFF;
    };
    pub const io = struct {
        pub const start: u16 = 0xFF00;
        pub const end: u16 = 0xFF7F;
        pub const size: usize = end - start + 1;
    };
    pub const hram = struct {
        pub const start: u16 = 0xFF80;
        pub const end: u16 = 0xFFFE;
        pub const size: usize = end - start + 1;
    };
    pub const ie_reg = 0xFFFF;
};

pub const Io = struct {
    pub const joyp: u16 = 0xFF00;
    pub const sb: u16 = 0xFF01;
    pub const sc: u16 = 0xFF02;
    pub const div: u16 = 0xFF04;
    pub const tima: u16 = 0xFF05;
    pub const tma: u16 = 0xFF06;
    pub const tac: u16 = 0xFF07;
    pub const if_reg: u16 = 0xFF0F;

    // Audio
    pub const nr10: u16 = 0xFF10;
    pub const nr11: u16 = 0xFF11;
    pub const nr12: u16 = 0xFF12;
    pub const nr14: u16 = 0xFF14;
    pub const nr21: u16 = 0xFF16;
    pub const nr22: u16 = 0xFF17;
    pub const nr24: u16 = 0xFF19;
    pub const nr30: u16 = 0xFF1A;
    pub const nr31: u16 = 0xFF1B;
    pub const nr32: u16 = 0xFF1C;
    pub const nr34: u16 = 0xFF1E;
    pub const nr41: u16 = 0xFF20;
    pub const nr42: u16 = 0xFF21;
    pub const nr43: u16 = 0xFF22;
    pub const nr44: u16 = 0xFF23;
    pub const nr50: u16 = 0xFF24;
    pub const nr51: u16 = 0xFF25;
    pub const nr52: u16 = 0xFF26;

    // LCD
    pub const lcdc: u16 = 0xFF40;
    pub const stat: u16 = 0xFF41;
    pub const scy: u16 = 0xFF42;
    pub const scx: u16 = 0xFF43;
    pub const ly: u16 = 0xFF44;
    pub const lyc: u16 = 0xFF45;
    pub const dma: u16 = 0xFF46;
    pub const bgp: u16 = 0xFF47;
    pub const obp0: u16 = 0xFF48;
    pub const obp1: u16 = 0xFF49;
    pub const wy: u16 = 0xFF4A;
    pub const wx: u16 = 0xFF4B;
};

pub const Interrupt = struct {
    pub const vblank: u16 = 0x0040;
    pub const lcd: u16 = 0x0048;
    pub const timer: u16 = 0x0050;
    pub const serial: u16 = 0x0058;
    pub const joypad: u16 = 0x0060;
};

pub const Boot = struct {
    pub const pc: u16 = 0x0100;
    pub const sp: u16 = 0xFFFE;
};

pub const Mbc = struct {
    pub const rom_bank_size: usize = 0x4000;
    pub const ram_bank_size: usize = 0x2000;
};

pub const Header = struct {
    pub const title_start: usize = 0x0134;
    pub const title_end: usize = 0x0144;
    pub const cart_type: usize = 0x0147;
    pub const rom_size: usize = 0x0148;
    pub const ram_size: usize = 0x0149;
};
