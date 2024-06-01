#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <arm_math.h>
#include <arm_const_structs.h>
#include <math.h>
#include "analyzer.h"

#define _fftSize 1024
const uint32_t fftSize = _fftSize;

float32_t fftBuf[_fftSize * 2];

float32_t
hamming (int i, int nn)
{
  const double _M_PI = 3.14159265358979323846;
  return ( 0.54 - 0.46 * cos (2.0 * _M_PI * (double)i / (double)(nn-1)) );
}

float32_t w[_fftSize]; // Hamming window
arm_cfft_instance_f32 s_cfft_f32_inst;

arm_status analyzer_init() {
  for(size_t i = 0; i < fftSize; ++i) {
    w[i] = hamming(i, fftSize);
  }
  arm_status status;
  status = arm_cfft_init_f32(&s_cfft_f32_inst, fftSize);
  return status;
}

static void process_chunk() {


  /* Process the data through the CFFT/CIFFT module */
  arm_cfft_f32(&s_cfft_f32_inst, fftBuf, 0/*inverse*/, 1/*doBitReverse*/);

  /* Process the data through the Complex Magnitude Module for
  calculating the magnitude at each bin */
  //arm_cmplx_mag_f32(testInput_f32_10khz, testOutput, fftSize);

  /* Calculates maxValue and returns corresponding BIN value */
  //arm_max_f32(testOutput, fftSize, &maxValue, &testIndex);
}

void analyzer_process(void* _buf, uint32_t size) {
  // Append data to ringbuf
  process_chunk();
}
