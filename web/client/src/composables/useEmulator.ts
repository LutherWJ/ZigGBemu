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
  onFrameReady: (cb: () => void) => void;
  init: (canvas: HTMLCanvasElement) => Promise<void>;
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
  
  let sharedMemory: WebAssembly.Memory | null = null;
  let romPtr: number = 0;
  let frameReadyCallback: (() => void) | null = null;

  const init = async (canvas: HTMLCanvasElement) => {
    try {
      console.log("[MAIN] Initializing Emulator...");
      const offscreen = canvas.transferControlToOffscreen();

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
            console.log("[MAIN] Worker Ready. ROM Pointer:", data.romPtr);
            isLoaded.value = true;
            romPtr = data.romPtr;
            break;
          case 'ROM_LOADED':
            console.log("[MAIN] ROM Loaded. Mapping Pointers:", data);
            isRomLoaded.value = true;
            if (sharedMemory) {
                const buffer = sharedMemory.buffer;
                memory.value = {
                    vram: new Uint8Array(buffer, data.vramPtr, 0x2000),
                    wram0: new Uint8Array(buffer, data.wram0Ptr, 0x1000),
                    wram1: new Uint8Array(buffer, data.wram1Ptr, 0x1000),
                    hram: new Uint8Array(buffer, data.hramPtr, 0x7F),
                    oam: new Uint8Array(buffer, data.oamPtr, 0xA0),
                    frameBuffer: new Uint32Array(buffer, data.frameBufferPtr, 160 * 144),
                };
            }
            break;
          case 'FRAME_READY':
            if (frameReadyCallback) frameReadyCallback();
            break;
          case 'ERROR':
            console.error("[MAIN] Worker Error:", data);
            error.value = data;
            break;
          case 'ZIG_LOG':
            console.log("[ZIG]", data);
            break;
          case 'ZIG_PANIC':
            console.error("[ZIG PANIC]", data);
            error.value = `Panic: ${data}`;
            break;
        }
      };

      // 4. Send the compiled module and shared memory to the worker
      worker.postMessage({
        type: 'INIT',
        data: {
          module: wasmModule,
          memory: sharedMemory,
          canvas: offscreen
        }
      }, [offscreen]);

    } catch (e: any) {
      console.error("[MAIN] Failed to load WASM/Worker:", e);
      error.value = e.message;
    }
  };

  const loadRom = async (file: File) => {
    if (!workerRef.value || !sharedMemory || !romPtr) {
        console.error("[MAIN] Cannot load ROM: Worker or Memory not ready");
        return;
    }

    const buffer = await file.arrayBuffer();
    const romData = new Uint8Array(buffer);

    console.log(`[MAIN] Loading ROM into buffer at 0x${romPtr.toString(16)}...`);
    const wasmMemory = new Uint8Array(sharedMemory.buffer, romPtr, romData.length);
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

  const onFrameReady = (cb: () => void) => {
    frameReadyCallback = cb;
  };

  return {
    isLoaded,
    isRomLoaded,
    memory,
    error,
    worker: workerRef,
    onFrameReady,
    init,
    loadRom,
    runFrame,
    step
  };
}
