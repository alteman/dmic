#pragma once
#include <stdint.h>
#include <arm_math_types.h>

#define STRIP_NODE              DT_ALIAS(led_strip)

#if DT_NODE_HAS_PROP(DT_ALIAS(led_strip), chain_length)
#define STRIP_NUM_PIXELS        DT_PROP(DT_ALIAS(led_strip), chain_length)
#else
#error Unable to determine length of LED strip
#endif

#define DELAY_TIME K_MSEC(CONFIG_SAMPLE_LED_UPDATE_DELAY)

#define RGB(_r, _g, _b) { .r = (_r), .g = (_g), .b = (_b) }

typedef struct {
  uint8_t r, g, b;
} __attribute__((packed)) pixel_t;

int neopixel_init();
void neopixel_update(const pixel_t* pixels, size_t cnt);
void neopixel_fromFloats(const float32_t* fpixels);
