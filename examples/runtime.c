#include "robot_mmio.h"
#include <stddef.h>

static volatile unsigned char source[4] = {1, 2, 3, 4};

/* Allocate a fresh page so the fixture exercises the actual WASM grow path. */
static volatile unsigned char *allocate_page(void) {
    const size_t previous = __builtin_wasm_memory_grow(0, 1);
    if (previous == (size_t)-1) __builtin_trap();
    return (volatile unsigned char *)(uintptr_t)(previous * 65536u);
}

void robot_main_v1(void) {
    volatile unsigned char *copy = allocate_page();
    for (size_t i = 0; i < sizeof source; ++i) copy[i] = source[i];
    if (copy[0] != 1 || copy[3] != 4 || __builtin_wasm_memory_size(0) != 2)
        __builtin_trap();
    ROBOT_TARGET = 7;
    ROBOT_COMMAND = copy[1] + copy[2];
}

