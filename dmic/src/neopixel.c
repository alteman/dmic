#define LOG_LEVEL 4
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(neopixel);

#include <zephyr/kernel.h>
#include <zephyr/drivers/led_strip.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/sys/util.h>

#include "neopixel.h"

#define STRIP_NODE              DT_ALIAS(led_strip)

#if DT_NODE_HAS_PROP(DT_ALIAS(led_strip), chain_length)
#define STRIP_NUM_PIXELS        DT_PROP(DT_ALIAS(led_strip), chain_length)
#else
#error Unable to determine length of LED strip
#endif

#define DELAY_TIME K_MSEC(CONFIG_SAMPLE_LED_UPDATE_DELAY)

#define RGB(_r, _g, _b) { .r = (_r), .g = (_g), .b = (_b) }

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
    return -1;
  } else {
    LOG_ERR("LED strip device %s is not ready", strip->name);
    return 0;
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
