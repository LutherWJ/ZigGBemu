import { ref, shallowRef, type Ref } from 'vue';

export interface EmulatorMemory {
  vram: Uint8Array;
  wram0: Uint8Array;
  wram1: Uint8Array;
  hram: Uint8Array;
  oam: Uint8Array;
  frameBuffer: Uint32Array;
}

export interface EmulatorState {
  isLoaded: Ref<boolean>;
  isRomLoaded: Ref<boolean>;
  memory: Ref<EmulatorMemory | null>;
  error: Ref<string | null>;
  worker: Ref<Worker | null>;
  init: () => Promise<void>;
  loadRom: (file: File) => Promise<void>;
  runFrame: () => void;
  step: () => void;
}

export function useEmulator(): EmulatorState {
  const isLoaded = ref(false);
  const isRomLoaded = ref(false);
  const memory = shallowRef<EmulatorMemory | null>(null);
  const error = ref<string | null>(null);
  const workerRef = shallowRef<Worker | null>(null);
  
  let wasmExports: any = null;
  let sharedMemory: WebAssembly.Memory | null = null;

  const init = async () => {
    try {
      // 1. Create Shared Memory (64MB = 1024 pages)
      sharedMemory = new WebAssembly.Memory({
        initial: 1024,
        maximum: 1024,
        shared: true
      });

      // 2. Fetch and compile the WASM module
      const response = await fetch("/ziggbemu.wasm");
      const wasmModule = await WebAssembly.compileStreaming(response);

      // 3. Initialize the Web Worker
      const worker = new Worker(new URL('../worker/emulator.worker.ts', import.meta.url), {
        type: 'module'
      });
      workerRef.value = worker;

      worker.onmessage = (e) => {
        const { type, data } = e.data;
        switch (type) {
          case 'READY':
            isLoaded.value = true;
            break;
          case 'ROM_LOADED':
            isRomLoaded.value = true;
            refreshMemoryViews();
            break;
          case 'ERROR':
            error.value = data;
            break;
        }
      };

      // 4. Send the compiled module and shared memory to the worker
      worker.postMessage({
        type: 'INIT',
        data: {
          module: wasmModule,
          memory: sharedMemory
        }
      });

      // Instantiate on main thread for pointers
      const instance = await WebAssembly.instantiate(wasmModule, {
        env: { memory: sharedMemory }
      });
      wasmExports = instance.exports;

    } catch (e: any) {
      console.error("Failed to load WASM/Worker:", e);
      error.value = e.message;
    }
  };

  const refreshMemoryViews = () => {
    if (!wasmExports || !sharedMemory) return;
    
    const buffer = sharedMemory.buffer;
    
    memory.value = {
      vram: new Uint8Array(buffer, wasmExports.get_vram_ptr(), 0x2000),
      wram0: new Uint8Array(buffer, wasmExports.get_wram0_ptr(), 0x1000),
      wram1: new Uint8Array(buffer, wasmExports.get_wram1_ptr(), 0x1000),
      hram: new Uint8Array(buffer, wasmExports.get_hram_ptr(), 0x7F),
      oam: new Uint8Array(buffer, wasmExports.get_oam_ptr(), 0xA0),
      frameBuffer: new Uint32Array(buffer, wasmExports.get_frame_buffer_ptr(), 160 * 144),
    };
  };

  const loadRom = async (file: File) => {
    if (!workerRef.value || !wasmExports || !sharedMemory) return;

    const buffer = await file.arrayBuffer();
    const romData = new Uint8Array(buffer);

    const ptr = wasmExports.get_rom_buffer_ptr();
    const wasmMemory = new Uint8Array(sharedMemory.buffer, ptr, romData.length);
    wasmMemory.set(romData);

    workerRef.value.postMessage({
      type: 'LOAD_ROM',
      data: { romSize: romData.length }
    });
  };

  const runFrame = () => {
    if (workerRef.value && isRomLoaded.value) {
      workerRef.value.postMessage({ type: 'RUN_FRAME' });
    }
  };

  const step = () => {
    if (workerRef.value && isRomLoaded.value) {
      workerRef.value.postMessage({ type: 'STEP' });
    }
  };

  return {
    isLoaded,
    isRomLoaded,
    memory,
    error,
    worker: workerRef,
    init,
    loadRom,
    runFrame,
    step
  };
}
