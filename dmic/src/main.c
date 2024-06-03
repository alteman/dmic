/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/audio/dmic.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/led.h>
#include <zephyr/logging/log.h>

#include "analyzer.h"
#include "neopixel.h"

LOG_MODULE_REGISTER(dmic_sample);

static const struct gpio_dt_spec led0_gpio __UNUSED = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
static const struct gpio_dt_spec led1_gpio = GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios);
static const struct gpio_dt_spec led2_gpio __UNUSED = GPIO_DT_SPEC_GET(DT_ALIAS(led2), gpios);

//#define MAX_SAMPLE_RATE  16000
#define MAX_SAMPLE_RATE  8000
#define SAMPLE_BIT_WIDTH 16
#define BYTES_PER_SAMPLE sizeof(int16_t)
/* Milliseconds to wait for a block to be read. */
#define READ_TIMEOUT     1000

#if 0
/* Size of a block for 100 ms of audio data. */
#define BLOCK_SIZE(_sample_rate, _number_of_channels) \
	(BYTES_PER_SAMPLE * (_sample_rate / 10) * _number_of_channels)

/* Driver will allocate blocks from this slab to receive audio data into them.
 * Application, after getting a given block from the driver and processing its
 * data, needs to free that block.
 */
// Aligned to fft size
#define ALIGN_UP(X, N) (X % N) ? X + (N - X % N) : X
#define MAX_BLOCK_SIZE   ALIGN_UP(BLOCK_SIZE(MAX_SAMPLE_RATE, 2), _fft_size)
#endif

//#define BLOCK_COUNT      MAX_SAMPLE_RATE / _fft_step
K_MEM_SLAB_DEFINE_STATIC(mem_slab, _fft_step * sizeof(int16_t), 64, 4);
//K_MEM_SLAB_DEFINE_STATIC(mem_slab, 6400, 4, 4);

static int do_pdm_transfer(const struct device *dmic_dev,
			   struct dmic_cfg *cfg)
{
	int ret;

	LOG_INF("PCM output rate: %u, channels: %u",
		cfg->streams[0].pcm_rate, cfg->channel.req_num_chan);

	ret = dmic_configure(dmic_dev, cfg);
	if (ret < 0) {
		LOG_ERR("Failed to configure the driver: %d", ret);
		return ret;
	}
	LOG_INF("dmic_configure done\n");

	ret = dmic_trigger(dmic_dev, DMIC_TRIGGER_START);
	if (ret < 0) {
		LOG_ERR("START trigger failed: %d", ret);
		return ret;
	}
	LOG_INF("dmic_trigger done\n");

	for (int i = 0; ; ++i) {
		void *buffer;
		uint32_t size;

		ret = dmic_read(dmic_dev, 0, &buffer, &size, READ_TIMEOUT);
		if (ret < 0) {
			LOG_ERR("%d - read failed: %d", i, ret);
			return ret;
		}
                gpio_pin_set_dt(&led1_gpio, i & 1);

                //LOG_INF("%d - got buffer %p of %u bytes", i, buffer, size);
		analyzer_process(buffer, size);
		//neopixel_update(NULL, 0);
		k_mem_slab_free(&mem_slab, buffer);
	}

	ret = dmic_trigger(dmic_dev, DMIC_TRIGGER_STOP);
	if (ret < 0) {
		LOG_ERR("STOP trigger failed: %d", ret);
		return ret;
	}

	return ret;
}

static void modules_init() {
  LOG_INF("modules_init\n");
  if (analyzer_init() != ARM_MATH_SUCCESS) {
    for(;;) {
      LOG_ERR("analyzer_init failed!\n");
      k_sleep(K_SECONDS(1));
    }
  }
  LOG_INF("analyzer_init done\n");

  if (neopixel_init() != 0) {
    for(;;) {
      LOG_ERR("neopixel_init failed!\n");
      k_sleep(K_SECONDS(1));
    }
  }
  LOG_INF("modules_init done\n");
}


#include <zephyr/drivers/counter.h>

const struct device *const timer0 = DEVICE_DT_GET(DT_NODELABEL(timer0));
void timer_init() {
        /* Grab the timer. */
        if (!device_is_ready(timer0)) {
                printk("%s: device not ready.\n", timer0->name);
                return;
        }

        /* Apparently there's no API to configure a frequency at
         * runtime, so live with whatever we get.
         */
        uint64_t ref_hz = counter_get_frequency(timer0);
        if (ref_hz == 0) {
                printk("Timer %s has no fixed frequency\n",
                        timer0->name);
                return;
        }

        uint32_t top = counter_get_top_value(timer0);
        if (top != UINT32_MAX) {
                printk("Timer %s wraps at %u (0x%08x) not at 32 bits\n",
                       timer0->name, top, top);
                return;
        }

	int rc = counter_start(timer0);
        printk("Start %s: %d\n", timer0->name, rc);
        //uint32_t t0, t1;
        //rc = counter_get_value(timer0, &t0);
        //k_sleep(K_SECONDS(1));
        //rc = counter_get_value(timer0, &t1);
	printk("Timer freq: %u\n", 160000000/*t1 - t0*/);
}

int main(void)
{
	LOG_ERR("LOG_ERR\n");
	printk("Start main\n");
	LOG_INF("LOG_INF");
	timer_init();
	const struct device *const dmic_dev = DEVICE_DT_GET(DT_NODELABEL(dmic_dev));
	int ret;
  
	if (!gpio_is_ready_dt(&led1_gpio)) {
                printk("%s: device not ready.\n", led1_gpio.port->name);
                return 0;
        }
	LOG_ERR("led1_gpio\n");
	k_sleep(K_SECONDS(1));
        gpio_pin_configure_dt(&led1_gpio, GPIO_OUTPUT_INACTIVE);
	modules_init();
	gpio_pin_set_dt(&led1_gpio, 1);
	LOG_ERR("Start dmic\n");
	k_sleep(K_SECONDS(1));

	LOG_INF("DMIC sample");

	if (!device_is_ready(dmic_dev)) {
		LOG_ERR("%s is not ready", dmic_dev->name);
		return 0;
	}

	struct pcm_stream_cfg stream = {
		.pcm_width = SAMPLE_BIT_WIDTH,
		.mem_slab  = &mem_slab,
	};
	struct dmic_cfg cfg = {
		.io = {
			/* These fields can be used to limit the PDM clock
			 * configurations that the driver is allowed to use
			 * to those supported by the microphone.
			 */
			.min_pdm_clk_freq = 1000000,
			.max_pdm_clk_freq = 3500000,
			.min_pdm_clk_dc   = 40,
			.max_pdm_clk_dc   = 60,
		},
		.streams = &stream,
		.channel = {
			.req_num_streams = 1,
		},
	};

	cfg.channel.req_num_chan = 1;
	cfg.channel.req_chan_map_lo =
		dmic_build_channel_map(0, 0, PDM_CHAN_LEFT);
	cfg.streams[0].pcm_rate = MAX_SAMPLE_RATE;
	//cfg.streams[0].block_size =
	//	BLOCK_SIZE(cfg.streams[0].pcm_rate, cfg.channel.req_num_chan);
	cfg.streams[0].block_size =
		mem_slab.info.block_size;

	ret = do_pdm_transfer(dmic_dev, &cfg);
	if (ret < 0) {
		return 0;
	}
#if 0
	cfg.channel.req_num_chan = 2;
	cfg.channel.req_chan_map_lo =
		dmic_build_channel_map(0, 0, PDM_CHAN_LEFT) |
		dmic_build_channel_map(1, 0, PDM_CHAN_RIGHT);
	cfg.streams[0].pcm_rate = MAX_SAMPLE_RATE;
	cfg.streams[0].block_size =
		BLOCK_SIZE(cfg.streams[0].pcm_rate, cfg.channel.req_num_chan);

	ret = do_pdm_transfer(dmic_dev, &cfg, 2 * BLOCK_COUNT);
	if (ret < 0) {
		return 0;
	}
#endif
	LOG_INF("Exiting");
	return 0;
}
