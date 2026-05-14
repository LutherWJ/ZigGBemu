<script setup lang="ts">
import { ref, watch, onMounted } from 'vue';
import { useMemView } from '../composables/useMemView';
import type { EmulatorMemory } from '../composables/useEmulator';

const props = defineProps<{
    memory: EmulatorMemory | null;
    refreshTrigger: number;
}>();

const regions = [
    { label: 'VRAM', start: 0x8000, key: 'vram' },
    { label: 'WRAM0', start: 0xC000, key: 'wram0' },
    { label: 'WRAM1', start: 0xD000, key: 'wram1' },
    { label: 'HRAM', start: 0xFF80, key: 'hram' },
    { label: 'OAM', start: 0xFE00, key: 'oam' },
] as const;

const selectedRegionKey = ref<keyof EmulatorMemory>('vram');
const currentRegion = regions.find(r => r.key === selectedRegionKey.value)!;

const memView = useMemView(new Uint8Array(0), 0);

const updateView = () => {
    if (!props.memory) return;
    const region = regions.find(r => r.key === selectedRegionKey.value)!;
    const buf = props.memory[region.key] as Uint8Array;
    memView.updateBuffer(buf, region.start);
};

watch(() => props.refreshTrigger, () => {
    if (!props.memory) return;
    memView.makeTable();
});

watch(selectedRegionKey, () => {
    updateView();
});

watch(() => props.memory, (newMem) => {
    if (newMem) updateView();
}, { immediate: true });

onMounted(() => {
    updateView();
});

function formatByte(byte: number) {
    return byte.toString(16).padStart(2, '0').toUpperCase();
}
</script>

<template>
    <div class="memory-viewer">
        <div class="header">
            <h3>Memory Viewer</h3>
            <div class="controls">
                <select v-model="selectedRegionKey">
                    <option v-for="r in regions" :key="r.key" :value="r.key">
                        {{ r.label }} (0x{{ r.start.toString(16).toUpperCase() }})
                    </option>
                </select>
                <div class="pagination">
                    <button @click="memView.prevPage">Prev</button>
                    <button @click="memView.nextPage">Next</button>
                </div>
            </div>
        </div>

        <div class="table-container" v-if="props.memory">
            <table>
                <thead>
                    <tr>
                        <th>Addr</th>
                        <th v-for="x in memView.memTable.value.xAxis" :key="x">{{ x }}</th>
                    </tr>
                </thead>
                <tbody>
                    <tr v-for="(row, i) in memView.memTable.value.table" :key="i">
                        <td class="addr">{{ memView.memTable.value.yAxis[i] }}</td>
                        <td v-for="(byte, j) in row" :key="j" :class="{ zero: byte === 0 }">
                            {{ formatByte(byte) }}
                        </td>
                    </tr>
                </tbody>
            </table>
        </div>
        <div v-else class="placeholder">
            Waiting for emulator to load...
        </div>
    </div>
</template>

<style scoped>
.memory-viewer {
    background: #1e1e1e;
    color: #ccc;
    padding: 1rem;
    border-radius: 8px;
    font-family: 'Courier New', Courier, monospace;
}

.header {
    display: flex;
    justify-content: space-between;
    align-items: center;
    margin-bottom: 1rem;
}

h3 {
    margin: 0;
    color: #eee;
}

.controls {
    display: flex;
    gap: 1rem;
}

select {
    background: #333;
    color: #eee;
    border: 1px solid #444;
    padding: 2px 5px;
}

.pagination button {
    background: #333;
    color: #eee;
    border: 1px solid #444;
    cursor: pointer;
    padding: 2px 8px;
}

.pagination button:hover {
    background: #444;
}

.table-container {
    overflow-x: auto;
}

table {
    border-collapse: collapse;
    font-size: 0.85rem;
    width: 100%;
}

th, td {
    padding: 2px 4px;
    text-align: center;
    border: 1px solid #333;
}

th {
    background: #2d2d2d;
    color: #888;
}

.addr {
    color: #569cd6;
    font-weight: bold;
    text-align: left;
    padding-right: 10px;
}

.zero {
    color: #444;
}

.placeholder {
    padding: 2rem;
    text-align: center;
    color: #666;
}
</style>
