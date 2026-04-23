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
                            // TextDecoder cannot decode directly from SharedArrayBuffer
                            const sharedBuf = new Uint8Array(memory.buffer, ptr, len);
                            const nonSharedBuf = new Uint8Array(sharedBuf); 
                            const str = new TextDecoder().decode(nonSharedBuf);
                            logToMain(str);
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

        case 'RUN_FRAME':
            if (!wasm || !ctx || !imageData || !memory) return;
            wasm.run_frame();
            
            const ptr = wasm.get_frame_buffer_ptr();
            const source = new Uint8Array(memory.buffer, ptr, 160 * 144 * 4);

            // Check if buffer has any data
            let hasData = false;
            for (let i = 0; i < 400; i++) {
                if (source[i] !== 0) {
                    hasData = true;
                    break;
                }
            }
            if (hasData) {
                console.log("[WORKER] Frame ready with data. First 16 bytes:", source.slice(0, 16));
            } else {
                console.log("[WORKER] Frame ready but empty (all zeros).");
            }

            imageData.data.set(source);
            ctx.putImageData(imageData, 0, 0);

            self.postMessage({ type: 'FRAME_READY' });
            break;

        case 'STEP':
            if (!wasm) return;
            wasm.step();
            self.postMessage({ type: 'STEP_COMPLETE' });
            break;
    }
};
