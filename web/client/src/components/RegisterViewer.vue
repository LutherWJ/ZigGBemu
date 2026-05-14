<script setup lang="ts">
import { ref, watch } from 'vue';
import type { EmulatorRegisters } from '../composables/useEmulator';

const props = defineProps<{
    registers: EmulatorRegisters;
    clock: number;
    refreshTrigger: number;
}>();

const localRegisters = ref({ ...props.registers });
const localClock = ref(props.clock);

watch(() => props.refreshTrigger, () => {
    localRegisters.value = { ...props.registers };
    localClock.value = props.clock;
});

function formatHex(val: number, len: number) {
    return val.toString(16).toUpperCase().padStart(len, '0');
}
</script>

<template>
    <div class="register-viewer">
        <h3>CPU Registers</h3>
        <div class="reg-grid">
            <div class="reg"><span>PC:</span> 0x{{ formatHex(localRegisters.pc, 4) }}</div>
            <div class="reg"><span>SP:</span> 0x{{ formatHex(localRegisters.sp, 4) }}</div>
            <div class="reg"><span>AF:</span> 0x{{ formatHex(localRegisters.a, 2) }}{{ formatHex(localRegisters.f, 2) }}</div>
            <div class="reg"><span>BC:</span> 0x{{ formatHex(localRegisters.b, 2) }}{{ formatHex(localRegisters.c, 2) }}</div>
            <div class="reg"><span>DE:</span> 0x{{ formatHex(localRegisters.d, 2) }}{{ formatHex(localRegisters.e, 2) }}</div>
            <div class="reg"><span>HL:</span> 0x{{ formatHex(localRegisters.h, 2) }}{{ formatHex(localRegisters.l, 2) }}</div>
            <div class="reg clock"><span>Clock:</span> {{ localClock }}</div>
        </div>
    </div>
</template>

<style scoped>
.register-viewer {
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

.reg-grid {
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
    color: #569cd6;
    font-weight: bold;
    margin-right: 0.4rem;
}

.clock {
    grid-column: span 2;
    border-top: 1px solid #444;
    margin-top: 0.2rem;
    padding-top: 0.5rem;
}
</style>
