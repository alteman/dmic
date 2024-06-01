#pragma once
#include <stdint.h>

typedef struct {
  uint8_t r, g, b;
} __attribute__((packed)) pixel_t;

int neopixel_init();
void neopixel_update(const pixel_t* pixels, size_t cnt);
