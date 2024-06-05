#pragma once

#include <stdint.h>

#define NUM_ADC_READS 32

int32_t adc_read_vbat();
int adc_test();
int adc_init();
