<script setup lang="ts">
import { ref, watch } from 'vue';
import type { PpuState } from '../composables/useEmulator';

const props = defineProps<{
    ppu: PpuState;
    refreshTrigger: number;
}>();

const localPpu = ref({ ...props.ppu });

watch(() => props.refreshTrigger, () => {
    localPpu.value = { ...props.ppu };
});

function formatHex(val: number, len: number) {
    return (val || 0).toString(16).toUpperCase().padStart(len, '0');
}
</script>

<template>
    <div class="ppu-viewer">
        <h3>PPU State</h3>
        <div class="ppu-grid">
            <div class="reg"><span>LCDC:</span> 0x{{ formatHex(localPpu.lcdc, 2) }}</div>
            <div class="reg"><span>STAT:</span> 0x{{ formatHex(localPpu.stat, 2) }}</div>
            <div class="reg"><span>SCY:</span> 0x{{ formatHex(localPpu.scy, 2) }}</div>
            <div class="reg"><span>SCX:</span> 0x{{ formatHex(localPpu.scx, 2) }}</div>
            <div class="reg"><span>LY:</span> 0x{{ formatHex(localPpu.ly, 2) }}</div>
            <div class="reg"><span>LYC:</span> 0x{{ formatHex(localPpu.lyc, 2) }}</div>
            <div class="reg"><span>WX:</span> 0x{{ formatHex(localPpu.wx, 2) }}</div>
            <div class="reg"><span>WY:</span> 0x{{ formatHex(localPpu.wy, 2) }}</div>
        </div>
    </div>
</template>

<style scoped>
.ppu-viewer {
    background: #1e1e1e;
    color: #ccc;
    padding: 1rem;
    border-radius: 8px;
    font-family: 'Courier New', Courier, monospace;
}

h3 {
    margin: 0 0 1rem 0;
    color: #eee;
    font-size: 1rem;
}

.ppu-grid {
    display: grid;
    grid-template-columns: repeat(2, 1fr);
    gap: 0.5rem;
}

.reg {
    background: #2d2d2d;
    padding: 0.3rem 0.6rem;
    border-radius: 4px;
    font-size: 0.9rem;
}

.reg span {
    color: #c586c0;
    font-weight: bold;
    margin-right: 0.4rem;
}
</style>
