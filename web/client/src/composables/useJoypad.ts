import { onMounted, onUnmounted } from 'vue';
import type { EmulatorState } from './useEmulator';

export function useJoypad(emulator: EmulatorState) {
    const keyMap: Record<string, number> = {
        'KeyJ': 0,      // A
        'KeyK': 1,      // B
        'ShiftRight': 2, // Select
        'Enter': 3,     // Start
        'KeyD': 4,      // Right
        'KeyA': 5,      // Left
        'KeyW': 6,      // Up
        'KeyS': 7,      // Down
    };

    const handleKeyDown = (e: KeyboardEvent) => {
        if (keyMap[e.code] !== undefined) {
            e.preventDefault();
            emulator.setButton(keyMap[e.code], true);
        }
    };

    const handleKeyUp = (e: KeyboardEvent) => {
        if (keyMap[e.code] !== undefined) {
            e.preventDefault();
            emulator.setButton(keyMap[e.code], false);
        }
    };

    onMounted(() => {
        window.addEventListener('keydown', handleKeyDown);
        window.addEventListener('keyup', handleKeyUp);
    });

    onUnmounted(() => {
        window.removeEventListener('keydown', handleKeyDown);
        window.removeEventListener('keyup', handleKeyUp);
    });
}
