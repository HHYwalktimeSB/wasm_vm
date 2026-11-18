#include "robot_mmio.h"

static volatile uint32_t command = 42;

void robot_main_v1(void) {
    ROBOT_TARGET = 7;
    ROBOT_COMMAND = command;
    while (ROBOT_STATUS == 1) {}
    if (ROBOT_STATUS != 2 || ROBOT_RESPONSE != (42u ^ 0xa5u))
        __builtin_trap();
}

