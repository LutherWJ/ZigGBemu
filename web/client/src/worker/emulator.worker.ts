// Game Boy Emulator Web Worker
let wasm: any = null;
let memory: WebAssembly.Memory | null = null;
let ctx: OffscreenCanvasRenderingContext2D | null = null;
let imageData: ImageData | null = null;

const logToMain = (msg: string) => {
    self.postMessage({ type: 'ZIG_LOG', data: msg });
};

const panicToMain = (msg: string) => {
    self.postMessage({ type: 'ZIG_PANIC', data: msg });
};

self.onmessage = async (e: MessageEvent) => {
    const { type, data } = e.data;
    console.log(`[WORKER] Received Message: ${type}`);

    switch (type) {
        case 'INIT':
            try {
                memory = data.memory;
                const offscreen = data.canvas as OffscreenCanvas;
                ctx = offscreen.getContext('2d', { alpha: false });
                if (ctx) {
                    imageData = ctx.createImageData(160, 144);
                }

                console.log("[WORKER] Instantiating WASM...");
                const result = await WebAssembly.instantiate(data.module, {
                    env: {
                        memory: memory,
                        js_log_write: (ptr: number, len: number) => {
                            if (!memory) return;
                            const sharedBuf = new Uint8Array(memory.buffer, ptr, len);
                            const nonSharedBuf = new Uint8Array(sharedBuf); 
                            const str = new TextDecoder().decode(nonSharedBuf);
                            logToMain(str);
                        },
                        js_panic: (ptr: number, len: number) => {
                            if (!memory) return;
                            const sharedBuf = new Uint8Array(memory.buffer, ptr, len);
                            const nonSharedBuf = new Uint8Array(sharedBuf); 
                            const str = new TextDecoder().decode(nonSharedBuf);
                            panicToMain(str);
                        }
                    }
                });
                wasm = result.exports;
                console.log("[WORKER] WASM Instantiated successfully");

                // Send only the ROM pointer initially
                self.postMessage({ 
                    type: 'READY',
                    data: {
                        romPtr: wasm.get_rom_buffer_ptr()
                    }
                });
            } catch (err: any) {
                console.error("[WORKER] Fatal Init Error:", err);
                self.postMessage({ type: 'ERROR', data: err.message });
            }
            break;

        case 'LOAD_ROM':
            if (!wasm) return;
            console.log(`[WORKER] Initializing Emulator with ROM size: ${data.romSize}`);
            const res = wasm.init_emulator(data.romSize);
            if (res !== 0) {
                const errorMap: Record<number, string> = {
                    1: 'ROM too large (> 8MB)',
                    2: 'ROM too small (invalid header)',
                    3: 'Unsupported MBC type',
                    4: 'Out of memory in WASM',
                    5: 'Unknown emulator initialization error'
                };
                self.postMessage({ type: 'ERROR', data: errorMap[res] || 'Initialization failed' });
            } else {
                // Now that the instance exists, send all pointers back to main
                self.postMessage({ 
                    type: 'ROM_LOADED',
                    data: {
                        vramPtr: wasm.get_vram_ptr(),
                        wram0Ptr: wasm.get_wram0_ptr(),
                        wram1Ptr: wasm.get_wram1_ptr(),
                        hramPtr: wasm.get_hram_ptr(),
                        oamPtr: wasm.get_oam_ptr(),
                        frameBufferPtr: wasm.get_frame_buffer_ptr()
                    }
                });
            }
            break;

        case 'SET_BUTTON':
            if (wasm) {
                wasm.set_button(data.id, data.pressed);
            }
            break;

        case 'RUN_FRAME':
            if (!wasm || !ctx || !imageData || !memory) return;
            try {
                wasm.run_frame();
            } catch (err: any) {
                console.error("[WORKER] WASM Trap in run_frame:", err);
                const stack = err.stack || "No JS stack available";
                panicToMain(`${err.message}\n\nJS Stack Trace:\n${stack}`);
                return;
            }
            
            const ptr = wasm.get_frame_buffer_ptr();
            const source = new Uint8Array(memory.buffer, ptr, 160 * 144 * 4);

            imageData.data.set(source);
            ctx.putImageData(imageData, 0, 0);

            self.postMessage({ 
                type: 'FRAME_READY',
                data: {
                    clock: wasm.get_clock(),
                    joypad: wasm.get_joypad_state(),
                    registers: {
                        pc: wasm.get_reg_pc(),
                        sp: wasm.get_reg_sp(),
                        a: wasm.get_reg_a(),
                        f: wasm.get_reg_f(),
                        b: wasm.get_reg_b(),
                        c: wasm.get_reg_c(),
                        d: wasm.get_reg_d(),
                        e: wasm.get_reg_e(),
                        h: wasm.get_reg_h(),
                        l: wasm.get_reg_l(),
                    },
                    ppu: {
                        lcdc: wasm.get_ppu_lcdc(),
                        stat: wasm.get_ppu_stat(),
                        scy: wasm.get_ppu_scy(),
                        scx: wasm.get_ppu_scx(),
                        ly: wasm.get_ppu_ly(),
                        lyc: wasm.get_ppu_lyc(),
                        wx: wasm.get_ppu_wx(),
                        wy: wasm.get_ppu_wy(),
                    }
                }
            });
            break;

        case 'STEP':
            if (!wasm) return;
            try {
                wasm.step();
            } catch (err: any) {
                console.error("[WORKER] WASM Trap in step:", err);
                const stack = err.stack || "No JS stack available";
                panicToMain(`${err.message}\n\nJS Stack Trace:\n${stack}`);
                return;
            }
            self.postMessage({ 
                type: 'STEP_COMPLETE',
                data: {
                    clock: wasm.get_clock(),
                    joypad: wasm.get_joypad_state(),
                    registers: {
                        pc: wasm.get_reg_pc(),
                        sp: wasm.get_reg_sp(),
                        a: wasm.get_reg_a(),
                        f: wasm.get_reg_f(),
                        b: wasm.get_reg_b(),
                        c: wasm.get_reg_c(),
                        d: wasm.get_reg_d(),
                        e: wasm.get_reg_e(),
                        h: wasm.get_reg_h(),
                        l: wasm.get_reg_l(),
                    },
                    ppu: {
                        lcdc: wasm.get_ppu_lcdc(),
                        stat: wasm.get_ppu_stat(),
                        scy: wasm.get_ppu_scy(),
                        scx: wasm.get_ppu_scx(),
                        ly: wasm.get_ppu_ly(),
                        lyc: wasm.get_ppu_lyc(),
                        wx: wasm.get_ppu_wx(),
                        wy: wasm.get_ppu_wy(),
                    }
                }
            });
            break;
    }
};
