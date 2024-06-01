#pragma once

#include <arm_math.h>

arm_status analyzer_init();
void analyzer_process(void *buffer, uint32_t size);
