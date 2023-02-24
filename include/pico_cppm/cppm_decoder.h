#ifndef __PICO_CPPM_CPPM_DECODER_H__
#define __PICO_CPPM_CPPM_DECODER_H__

#include <stdint.h>

#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/pio.h"

#include "cppm_decoder.pio.h"

class CPPMDecoder {
 public:
  CPPMDecoder(uint cppm_gpio, PIO pio = pio0,
    uint32_t max_period_us = DEFAULT_MAX_PERIOD_US,
    double calibrated_min_us = DEFAULT_CALIBRATED_MIN_US,
    double calibrated_max_us = DEFAULT_CALIBRATED_MAX_US)
    : cppm_gpio(cppm_gpio), pio(pio),
    max_period_us(max_period_us),
    calibrated_min_us(calibrated_min_us),
    calibrated_max_us(calibrated_max_us) {
    clocks_per_us = clock_get_hz(clk_sys) / MICROS_PER_SEC;
  }
  // TODO: cleanup in destructor

  // Start PIO/DMA, which will begin populating channel values continuously
  void startListening();

  // Get the latest value for channel index ch, in range [-1, 1]
  double getChannelValue(uint ch);

  // TODO: calibration mode

 private:
  static constexpr uint32_t MICROS_PER_SEC = 1'000'000;

  static constexpr uint32_t DEFAULT_MAX_PERIOD_US = 2500;
  static constexpr double DEFAULT_CALIBRATED_MIN_US = 900;
  static constexpr double DEFAULT_CALIBRATED_MAX_US = 2200;

  uint cppm_gpio;

  PIO pio;
  uint pio_offset = 0;
  uint pio_sm = 0;

  int dma_channel = -1;
  volatile uint32_t dma_buffer[cppm_decoder_NUM_CHANNELS] = {0};
  int dma_loop_channel = -1;
  volatile uint32_t* dma_buffer_ptr = (uint32_t*)&dma_buffer;

  uint32_t clocks_per_us = 0;
  uint32_t max_period_count = 0;
  // Maximum pulse-to-pulse period before starting a new frame
  uint32_t max_period_us;
  // Channel period that maps to -1
  double calibrated_min_us;
  // Channel period that maps to 1
  double calibrated_max_us;

  void initPIO();
  void startDMA();
};

#endif
