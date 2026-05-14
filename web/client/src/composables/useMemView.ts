import { ref } from "vue";

const ROWS = 16;
const COLS = 16;
const AREA = ROWS * COLS;

const xAxisValues = [
  '0', '1', '2', '3', '4', '5', '6', '7', 
  '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'
] as const;

type MemTableXAxis = typeof xAxisValues; 

export interface MemTable {
    table: number[][];
    xAxis: MemTableXAxis;
    yAxis: string[];
}

export const useMemView = (initialBuf: Uint8Array, initialAddress: number) => {
    let buf = initialBuf;
    let startAddress = initialAddress;

    const memTable = ref<MemTable>({
        table: [],
        xAxis: xAxisValues,
        yAxis: [],
    });

    let offset = 0;

    function makeTable () {
        if (!buf) return;
        const baseAddress = offset; // Offset is relative to the buffer start
        for (let i = 0; i < ROWS; i++) {
            const address = baseAddress + (COLS * i);
            memTable.value.yAxis[i] = (startAddress + address).toString(16).padStart(4, '0').toUpperCase();
            memTable.value.table[i] = Array.from(buf.slice(address, address + COLS));
        }
    }

    function updateBuffer(newBuf: Uint8Array, newAddress: number) {
        buf = newBuf;
        startAddress = newAddress;
        offset = 0;
        makeTable();
    }

    function nextPage() {
        if (buf.length - AREA <= offset) return;
        offset += AREA;
        makeTable();
    }

    function prevPage() {
        if (offset <= 0) return;
        offset -= AREA;
        makeTable();
    }

    function jumpPage(address: number) {
        // address is absolute, we need relative to startAddress
        const rel = address - startAddress;
        if (rel < 0 || rel > buf.length - AREA) return;
        offset = rel;
        makeTable();
    }

    makeTable();

    return {
        memTable,
        nextPage,
        prevPage,
        jumpPage,
        updateBuffer,
        makeTable
    }
}

export type MemView = ReturnType<typeof useMemView>;

