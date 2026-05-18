<script setup lang="ts">
import { computed } from 'vue';

const props = defineProps<{
  state: number;
}>();

const buttons = [
  { label: 'A', bit: 0 },
  { label: 'B', bit: 1 },
  { label: 'SELECT', bit: 2 },
  { label: 'START', bit: 3 },
  { label: 'RIGHT', bit: 4 },
  { label: 'LEFT', bit: 5 },
  { label: 'UP', bit: 6 },
  { label: 'DOWN', bit: 7 },
];

const isPressed = (bit: number) => {
  return (props.state & (1 << bit)) !== 0;
};
</script>

<template>
  <div class="joypad-viewer">
    <h3>Joypad</h3>
    <div class="controls-grid">
      <div class="dpad">
        <div class="dpad-row">
            <div class="empty"></div>
            <div :class="['btn', { active: isPressed(6) }]">↑</div>
            <div class="empty"></div>
        </div>
        <div class="dpad-row">
            <div :class="['btn', { active: isPressed(5) }]">←</div>
            <div class="empty"></div>
            <div :class="['btn', { active: isPressed(4) }]">→</div>
        </div>
        <div class="dpad-row">
            <div class="empty"></div>
            <div :class="['btn', { active: isPressed(7) }]">↓</div>
            <div class="empty"></div>
        </div>
      </div>
      
      <div class="action-buttons">
        <div :class="['btn-round', { active: isPressed(1) }]">B</div>
        <div :class="['btn-round', { active: isPressed(0) }]">A</div>
      </div>
    </div>
    <div class="system-buttons">
      <div :class="['btn-pill', { active: isPressed(2) }]">SELECT</div>
      <div :class="['btn-pill', { active: isPressed(3) }]">START</div>
    </div>
  </div>
</template>

<style scoped>
.joypad-viewer {
  background: #222;
  color: #fff;
  padding: 10px;
  border-radius: 8px;
  font-family: monospace;
  width: fit-content;
}

h3 {
  margin: 0 0 10px 0;
  font-size: 14px;
  text-align: center;
}

.controls-grid {
    display: flex;
    gap: 20px;
    align-items: center;
    justify-content: center;
}

.dpad {
    display: flex;
    flex-direction: column;
}

.dpad-row {
    display: flex;
}

.btn, .empty {
    width: 25px;
    height: 25px;
    display: flex;
    align-items: center;
    justify-content: center;
    border: 1px solid #444;
    background: #333;
}

.btn.active {
    background: #00ff00;
    color: #000;
}

.action-buttons {
    display: flex;
    gap: 10px;
    transform: rotate(-15deg);
}

.btn-round {
    width: 30px;
    height: 30px;
    border-radius: 50%;
    display: flex;
    align-items: center;
    justify-content: center;
    border: 1px solid #444;
    background: #333;
    font-size: 12px;
}

.btn-round.active {
    background: #ff0000;
    color: #fff;
}

.system-buttons {
    display: flex;
    gap: 10px;
    margin-top: 15px;
    justify-content: center;
}

.btn-pill {
    width: 60px;
    height: 15px;
    border-radius: 10px;
    display: flex;
    align-items: center;
    justify-content: center;
    border: 1px solid #444;
    background: #333;
    font-size: 10px;
}

.btn-pill.active {
    background: #888;
    color: #000;
}

.empty {
    border: none;
    background: transparent;
}
</style>
