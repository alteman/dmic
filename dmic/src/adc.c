#include <zephyr/kernel.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
#include <zephyr/logging/log.h>

#include "adc.h"

LOG_MODULE_REGISTER(adc);

#if !DT_NODE_EXISTS(DT_PATH(zephyr_user)) || \
  !DT_NODE_HAS_PROP(DT_PATH(zephyr_user), io_channels)
#error "No suitable devicetree overlay specified"
#endif

#define DT_SPEC_AND_COMMA(node_id, prop, idx) \
  ADC_DT_SPEC_GET_BY_IDX(node_id, idx),

/* Data of ADC io-channels specified in devicetree. */
static const struct adc_dt_spec adc_channels[] = {
  DT_FOREACH_PROP_ELEM(DT_PATH(zephyr_user), io_channels,
    DT_SPEC_AND_COMMA)
};

static int16_t buf[NUM_ADC_READS];

struct adc_sequence sequence = {
  .buffer = &buf,
  /* buffer size in bytes, not number of samples */
  .buffer_size = sizeof(buf),
  //Optional
  //.calibrate = true,
};

static int32_t s_lastAdcVal;

int32_t adc_read_vbat() {
  int32_t acc = 0;
  for (size_t i = 0; i < NUM_ADC_READS; ++i) {
    int32_t v = buf[i];
    int rc = adc_raw_to_millivolts_dt(adc_channels, &v);
    if (rc < 0) {
      LOG_ERR("adc_raw_to_millivolts_dt failed\n");
    }
    acc += v;
  }
  acc /= NUM_ADC_READS;
  s_lastAdcVal = acc;
  printk("adc.input_ch: %u\n", adc_channels->channel_cfg.input_positive);
  adc_read_async(adc_channels->dev, &sequence, NULL);
  return s_lastAdcVal;
}

uint16_t test_buf[1] = {42};

const struct gpio_dt_spec measure_pin = GPIO_DT_SPEC_GET_OR(VBATT, power_gpios, {});
const struct device *const gpio0 = DEVICE_DT_GET(DT_NODELABEL(gpio0));

struct adc_sequence test_seq = {
  .buffer = test_buf,
  .buffer_size = sizeof(test_buf),
};

int adc_test() {
  LOG_INF("ADC test start\n");
  adc_sequence_init_dt(adc_channels, &test_seq);
  struct adc_channel_cfg cc = adc_channels->channel_cfg;
  struct adc_channel_cfg*c = &cc;
  LOG_INF("id=%i, gain=%i, reference=%i, acquisition_time=%i, differential=%i, input_positive=%i, input_negative=%i\n",
    c->channel_id,
    c->gain,
    c->reference,
    c->acquisition_time,
    c->differential,
    c->input_positive,
    c->input_negative);
  struct adc_sequence* s = &test_seq;
  LOG_INF("options=%p, channels=%i, buffer=%p, buffer_size=%i, resolution=%i, oversampling=%i, calibrate=%i\n",
    s->options,
    s->channels,
    s->buffer,
    s->buffer_size,
    s->resolution,
    s->oversampling,
    s->calibrate);
  int rc;
  //c->acquisition_time = ADC_ACQ_TIME(ADC_ACQ_TIME_MICROSECONDS, 40); 
  rc = adc_channel_setup(adc_channels->dev, c);
  if (rc < 0) LOG_ERR("ADC channel setup failed (rc=%u)\n", rc);
  for(;;) {
    rc = gpio_pin_configure(gpio0, 14, GPIO_OUTPUT_INACTIVE);
    rc = adc_read(adc_channels->dev, &test_seq);
    rc = gpio_pin_configure(gpio0, 14, GPIO_INPUT);
    int32_t v = test_buf[0];
    int rc = adc_raw_to_millivolts_dt(adc_channels, &v);
    v = 1510 * v / 510;
    LOG_INF("ADC result: %u (raw %u) (rc=%u)\n", v, test_buf[0], rc);
    k_sleep(K_SECONDS(1));
  }
}


int adc_init() {
  for (size_t i = 0U; i < ARRAY_SIZE(adc_channels); i++) {
    if (!adc_is_ready_dt(adc_channels + i)) {
      LOG_ERR("ADC channel missing (%u)\n", i);
      return -1;
    }
    int rc = adc_channel_setup_dt(adc_channels + i);
    if (rc < 0) {
      LOG_ERR("ADC channel setup failed (%u)\n", i);
      return -1;
    }
    rc = adc_sequence_init_dt(adc_channels + i, &sequence);
    if (rc < 0) {
      LOG_ERR("ADC: Could not initalize sequnce (%u)", i);
      return -1;
    }
    rc = gpio_pin_configure(gpio0, 14, GPIO_OUTPUT_INACTIVE);
    if (rc < 0) {
      LOG_ERR("ADC: Could not configure the pull down pin (%u) rc=%i", i, rc);
      return -1;
    }
  }
  return 0;
}

