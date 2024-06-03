#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(analyzer);

#include <arm_math.h>
#include <arm_const_structs.h>
#include <math.h>
#include "analyzer.h"
#include "neopixel.h"

const uint32_t fftSize = _fft_size;
const uint32_t fftStep = _fft_step;

q15_t fftBuf[_fft_size];
q15_t fftOut[_fft_size];
//#define _c_bins (_fft_size / 2 + 1)
#define _c_bins (_fft_size / 8)
size_t cBins = _c_bins;
q15_t binMags[_c_bins];
float32_t fBinMags[_c_bins];

float32_t
hamming (int i, int nn)
{
  const double _M_PI = 3.14159265358979323846;
  return ( 0.54 - 0.46 * cos (2.0 * _M_PI * (double)i / (double)(nn-1)) );
}

float32_t w[_fft_size]; // Hamming window
arm_rfft_instance_q15 s_fft_inst;

arm_status analyzer_init() {
  for(size_t i = 0; i < fftSize; ++i) {
    w[i] = hamming(i, fftSize);
  }
  arm_status status;
  status = arm_rfft_init_q15(&s_fft_inst, fftSize, 0, 1);
  return status;
}

float32_t pixMag[STRIP_NUM_PIXELS] = {0};

static float32_t avg_maxval = 0;

#include <zephyr/drivers/counter.h>
extern const struct device *const timer0;
static const float beat_factor = 3.0;
#define last_e_len 16
static float32_t lastE[last_e_len];
size_t last_e_ptr = 0;
static void process_buf() {

  uint32_t t0, t1;
  int rc = counter_get_value(timer0, &t0);

  /* Process the data through the CFFT/CIFFT module */
  // arm_cfft_f32(&s_cfft_f32_inst, fftBuf, 0/*inverse*/, 1/*doBitReverse*/);

  arm_rfft_q15(&s_fft_inst, fftBuf, fftOut);

  /* Process the data through the Complex Magnitude Module for
  calculating the magnitude at each bin */
  //arm_cmplx_mag_f32(fftBuf, binMags, cBins);
  arm_cmplx_mag_q15(fftOut, binMags, cBins);

  arm_q15_to_float(binMags, fBinMags, cBins);

  float32_t e = 0;
  for (size_t i = 0; i < cBins; ++i) {
    e += fBinMags[i] * fBinMags[i];
  }

  float32_t avgE = 0;
  for (size_t i = 0; i < last_e_len; ++i) {
    //TODO: apply window?
    avgE += lastE[i];
  }
  bool beat = e * last_e_len > avgE * beat_factor;
  lastE[last_e_ptr++] = e;
  if (last_e_ptr >= last_e_len) {
    last_e_ptr = 0;
  }

  float32_t maxVal;
  uint32_t maxIdx;
  arm_absmax_f32(fBinMags, cBins, &maxVal, &maxIdx);
  if (maxVal > avg_maxval) {
    avg_maxval = (float32_t)(avg_maxval * 0.9 + maxVal * 0.1);
  } else {
    avg_maxval = (float32_t)(avg_maxval * 0.99 + maxVal * 0.01);
  }
  int prevPixIdx = -1;
  size_t cPix = 0;
  for (size_t i = 0; ; ++i) {
     size_t pixIdx = (i * STRIP_NUM_PIXELS) / _c_bins;
     if (pixIdx != prevPixIdx || i >= cBins) {
       // finalize the bin
       if (cPix) {
         float val = (pixMag[prevPixIdx] / (cPix * avg_maxval)) - 0.2;
	 if (beat) {
	   val += .4;
	 }
	 val /= 25;
         if (val < 0) {
           val = 0;
	 }
	 pixMag[prevPixIdx] = val; 
       }
       if (i >= cBins) {
         break;
       }
       pixMag[pixIdx] = 0;
       cPix = 0;
       prevPixIdx = pixIdx;
     }
     pixMag[pixIdx] += fBinMags[i];
     cPix++;
  }
  neopixel_fromFloats(pixMag);
  rc = counter_get_value(timer0, &t1);
  //LOG_INF("process_buf: dt=%u\n", t1 - t0);
}

void analyzer_process(const void* _newsamples, uint32_t cb) {
  static int16_t buf[_fft_size] = {0}; // previous sample
#if 0
  float e = 0;
  for (size_t i = 0; i < cb / 2; ++i) {
    int16_t v = ((int16_t*)_newsamples)[i];
    e += v * v;
  }
  LOG_INF("e=%08i", (int)e/1000000);
#endif
  if (cb != fftStep * 2) {
    return;
  }
  // shift existing data by fftStep
  memmove(buf, buf + fftStep, sizeof(int16_t) * (fftSize - fftStep));
  // append new chunk
  memcpy(buf + fftSize - fftStep, _newsamples, sizeof(int16_t) * fftStep);
  // apply window and convert to float
  for(size_t i = 0; i < fftSize; ++i) {
      fftBuf[i] = w[i] * buf[i];
  }
  process_buf();
}
