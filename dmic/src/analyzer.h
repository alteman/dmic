#pragma once

#include <arm_math.h>
#include "neopixel.h"

#define SAMPLE_RATE 16000
#define _fft_size 1024
#define _fft_step 256
#define MEL_BINS (STRIP_NUM_PIXELS / 2)

_Static_assert((_fft_size % _fft_step) == 0);

arm_status analyzer_init();
void analyzer_process(const void *buffer, uint32_t size);
