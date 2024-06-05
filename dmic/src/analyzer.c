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
#include "mfccdata.h"

const uint32_t fftSize = _fft_size;
const uint32_t fftStep = _fft_step;

q15_t fftBuf[_fft_size];
q15_t fftOut[_fft_size];
//#define _c_bins (_fft_size / 2 + 1)
#define _c_bins (_fft_size / 2)
size_t cBins = _c_bins;
q15_t binMags[_c_bins];
float32_t fBinMags[_c_bins];

#if 0
float32_t
hamming (int i, int nn)
{
  const double _M_PI = 3.14159265358979323846;
  return ( 0.54 - 0.46 * cos (2.0 * _M_PI * (double)i / (double)(nn-1)) );
}
#endif

float32_t w[_fft_size]; // Hamming window
arm_rfft_instance_q15 s_fft_inst;

const size_t f_lo = 40;
//const size_t f_hi = SAMPLE_RATE / 2;
const size_t f_hi = 2000;
const double fft_bin_hz = 1.0 * SAMPLE_RATE / _fft_size;

double mel(double f) {
  return 1000.0 / log10(2.0) * log10(1 + (f / 1000.0));
}

double invMel(double m) {
  return 1000.0 * (-1.0 + pow(2.0, m / 1000.0));
}

static uint16_t fftBinForMel[MEL_BINS + 1] = {0};

#define N_FIR_TAPS 4

float32_t firCoefs_f32[N_FIR_TAPS] = {1.f, -0.95f, 0.f, 0.f};
static q15_t firCoefs[N_FIR_TAPS];
static arm_fir_instance_q15 s_fir_inst;
static q15_t firState[N_FIR_TAPS + _fft_step];

arm_status analyzer_init() {
  arm_float_to_q15(firCoefs_f32, firCoefs, N_FIR_TAPS);
  arm_fir_init_q15(&s_fir_inst, N_FIR_TAPS, firCoefs, firState, fftStep);
  arm_hamming_f32(w, fftSize);
#if 0
  for(size_t i = 0; i < fftSize; ++i) {
    w[i] = hamming(i, fftSize);
  }
#endif
  // mel bin boundaries
  double mel_lo = mel(f_lo);
  double mel_hi = mel(f_hi);
  double mel_bin_size = (mel_hi - mel_lo) / MEL_BINS; 
  for (size_t i = 0; i < MEL_BINS + 1; ++i) {
    fftBinForMel[i] = (int)(round(invMel(mel_lo + mel_bin_size * i) / fft_bin_hz));
  }
  return arm_rfft_init_q15(&s_fft_inst, fftSize, 0, 1);
}

float32_t pixMag[STRIP_NUM_PIXELS] = {0};

static float32_t avg_maxval = 0;

#include <zephyr/drivers/counter.h>
extern const struct device *const timer0;
static const float beat_factor = 3.0;
#define last_e_len 16
static float32_t lastE[last_e_len];
static size_t last_e_ptr = 0;
static uint16_t mel_bins[MEL_BINS];
static q15_t firOut[_fft_size];
static void process_buf() {

  uint32_t t0, t1;
  int rc = counter_get_value(timer0, &t0);
  
  for (size_t i = 0; i < _fft_size / _fft_step; ++i) {
    arm_fir_q15(&s_fir_inst, fftBuf + _fft_step * i, firOut + _fft_step * i, fftStep);
  }
  arm_rfft_q15(&s_fft_inst, firOut, fftOut);

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
    avg_maxval = (avg_maxval * 0.9f + maxVal * 0.1f);
  } else {
    avg_maxval = (avg_maxval * 0.99f + maxVal * 0.01f);
  }
  float lgmax = log(avg_maxval);
  for (size_t i = 0; i < MEL_BINS; ++i) {
    uint16_t f0 = fftBinForMel[i], 
      f1 = fftBinForMel[i+1];
    float acc = 0;
    for(size_t j = f0; j < f1; ++j) {
      acc += fBinMags[j];
    }
    acc /= (f1 - f0);
    float v = logf(acc) - lgmax;
    mel_bins[i] = v;
    v = (v * 0.5f) + 0.4f;
    if (beat) {
      v += 1;
    }
    if (v > 0.5f) {
      v = 0.5f;
    }
    if (v < 0) {
      v = 0;
    }
    uint16_t pxidx = STRIP_NUM_PIXELS / 2 - 1 - i;
    pixMag[pxidx] = pixMag[STRIP_NUM_PIXELS - 1 - pxidx] = v;
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
