#define LOG_LEVEL 4
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(neopixel);

#include <zephyr/kernel.h>
#include <zephyr/drivers/led_strip.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/sys/util.h>

#include "neopixel.h"

static const struct led_rgb colors[] = {
        RGB(0x0f, 0x00, 0x00), /* red */
        RGB(0x00, 0x0f, 0x00), /* green */
        RGB(0x00, 0x00, 0x0f), /* blue */
};

static struct led_rgb pixels[STRIP_NUM_PIXELS];

static const struct device *const strip = DEVICE_DT_GET(STRIP_NODE);

int neopixel_init(void) {
  if (device_is_ready(strip)) {
    LOG_INF("Found LED strip device %s", strip->name);
    return 0;
  } else {
    LOG_ERR("LED strip device %s is not ready", strip->name);
    return -1;
  }
}

void neopixel_update(const pixel_t* _pixels, size_t cnt) {
  LOG_INF("Displaying pattern on strip");
  static size_t color = 0;
  static size_t cursor = 0;
  memset(&pixels, 0x00, sizeof(pixels));
  memcpy(&pixels[cursor], &colors[color], sizeof(struct led_rgb));

  int rc = led_strip_update_rgb(strip, pixels, STRIP_NUM_PIXELS);
  if (rc) {
    LOG_ERR("couldn't update strip: %d", rc);
  }

  color = (color + 1) % ARRAY_SIZE(colors);
  cursor = (cursor + 1) % ARRAY_SIZE(pixels);
}

void neopixel_fromFloats(const float32_t* fpix) {
  for (size_t i = 0; i < STRIP_NUM_PIXELS; ++i) {
    struct led_rgb *px = pixels + i;
    int val = (int)(fpix[i] * 256);
    if (val > 255) {
      val = 255;
    }
    px->r = px->g = px->b = val;
  }
  led_strip_update_rgb(strip, pixels, STRIP_NUM_PIXELS);
}
