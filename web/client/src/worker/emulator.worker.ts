// Game Boy Emulator Web Worker
let wasm: any = null;
let memory: WebAssembly.Memory | null = null;

self.onmessage = async (e: MessageEvent) => {
    const { type, data } = e.data;

    switch (type) {
        case 'INIT':
            try {
                memory = data.memory;
                const result = await WebAssembly.instantiate(data.module, {
                    env: {
                        memory: memory
                    }
                });
                wasm = result.exports;
                self.postMessage({ type: 'READY' });
            } catch (err: any) {
                self.postMessage({ type: 'ERROR', data: err.message });
            }
            break;

        case 'LOAD_ROM':
            if (!wasm) return;
            // The ROM was already copied into the shared buffer by the main thread
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
                self.postMessage({ type: 'ROM_LOADED' });
            }
            break;

        case 'RUN_FRAME':
            if (!wasm) return;
            wasm.run_frame();
            self.postMessage({ type: 'FRAME_READY' });
            break;

        case 'STEP':
            if (!wasm) return;
            wasm.step();
            self.postMessage({ type: 'STEP_COMPLETE' });
            break;
    }
};
