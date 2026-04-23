import { onMounted, onUnmounted, watch, type Ref } from 'vue';

export function useDisplay(canvasRef: Ref<HTMLCanvasElement | null>, frameBuffer: Ref<Uint32Array | null>, workerRef: Ref<Worker | null>) {
    let ctx: CanvasRenderingContext2D | null = null;
    let imageData: ImageData | null = null;
    let imageBuffer: Uint8ClampedArray | null = null;

    const render = () => {
        if (!ctx || !imageData || !imageBuffer || !frameBuffer.value) return;

        const data = imageBuffer;
        const source = frameBuffer.value;
        
        // Efficient copy
        const uint8Source = new Uint8Array(source.buffer, source.byteOffset, source.byteLength);
        data.set(uint8Source);

        ctx.putImageData(imageData, 0, 0);
    };

    const handleMessage = (e: MessageEvent) => {
        if (e.data.type === 'FRAME_READY') {
            render();
        }
    };

    onMounted(() => {
        if (!canvasRef.value) return;
        ctx = canvasRef.value.getContext('2d');
        if (!ctx) return;

        imageData = ctx.createImageData(160, 144);
        imageBuffer = imageData.data;
    });

    // Listen for worker changes (initialization)
    watch(workerRef, (newWorker, oldWorker) => {
        if (oldWorker) {
            oldWorker.removeEventListener('message', handleMessage);
        }
        if (newWorker) {
            newWorker.addEventListener('message', handleMessage);
        }
    }, { immediate: true });

    onUnmounted(() => {
        if (workerRef.value) {
            workerRef.value.removeEventListener('message', handleMessage);
        }
    });

    return { render };
}
