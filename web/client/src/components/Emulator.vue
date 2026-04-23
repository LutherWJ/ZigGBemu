<script setup lang="ts">
import { ref, onMounted, computed } from 'vue';
import { useEmulator } from '../composables/useEmulator';

const canvasRef = ref<HTMLCanvasElement | null>(null);
const emulator = useEmulator();

const handleFileUpload = (event: Event) => {
    const target = event.target as HTMLInputElement;
    if (target.files && target.files[0]) {
        emulator.loadRom(target.files[0]);
    }
};

onMounted(() => {
    if (canvasRef.value) {
        emulator.init(canvasRef.value);
    }
    emulator.onFrameReady(() => {
        if (isRunning.value) {
            animationId = requestAnimationFrame(loop);
        }
    });
});

const isRunning = ref(false);
let animationId: number | null = null;

const start = () => {
    if (isRunning.value) return;
    isRunning.value = true;
    loop();
};

const stop = () => {
    isRunning.value = false;
    if (animationId) cancelAnimationFrame(animationId);
};

const loop = () => {
    if (!isRunning.value) return;
    emulator.runFrame();
};
</script>

<template>
    <div class="emulator-container">
        <div class="controls">
            <input type="file" @change="handleFileUpload" accept=".gb,.gbc,.bin" />
            <button @click="start" :disabled="!emulator.isRomLoaded.value || isRunning">Run</button>
            <button @click="stop" :disabled="!isRunning">Stop</button>
            <span v-if="!emulator.isLoaded.value">Loading WASM...</span>
            <span v-else-if="emulator.isRomLoaded.value">ROM Loaded!</span>
            <span v-else>Ready for ROM.</span>
        </div>

        <div class="display">
            <canvas ref="canvasRef" width="160" height="144"></canvas>
        </div>

        <div v-if="emulator.error.value" class="error">
            Error: {{ emulator.error.value }}
        </div>
    </div>
</template>

<style scoped>
.emulator-container {
    display: flex;
    flex-direction: column;
    align-items: center;
    gap: 1rem;
    padding: 2rem;
}

.controls {
    display: flex;
    gap: 1rem;
    align-items: center;
}

.display {
    border: 4px solid #333;
    background: #000;
    image-rendering: pixelated;
}

canvas {
    width: 320px; /* Scaling for better visibility */
    height: 288px;
}

.error {
    color: #ff4444;
    background: #fee;
    padding: 0.5rem;
    border-radius: 4px;
}
</style>
