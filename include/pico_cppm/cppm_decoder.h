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
    uint32_t max_period_us = 2500,
    double calibrated_min_us = 1000,
    double calibrated_max_us = 2000)
    : cppm_gpio(cppm_gpio), pio(pio),
    max_period_us(max_period_us),
    calibrated_min_us(calibrated_min_us),
    calibrated_max_us(calibrated_max_us) {
    clocks_per_us = clock_get_hz(clk_sys) / MICROS_PER_SEC;
  }
  // TODO: cleanup in destructor

  // Start PIO/DMA, which will begin populating channel values continuously
  bool startListening();

  // Get the latest value for channel index ch, in range [-1, 1]
  double getChannelValue(uint ch);

  // Calibration adjusts the min/max duration across all channels, based on the durations seen
  // in samples taken by processCalibration
  void beginCalibration();
  void processCalibration();
  // Returns true and puts calibration into effect if the min/max spread is at least min_spread_us
  bool endCalibration(double min_spread_us = 500);

 private:
  static constexpr uint32_t MICROS_PER_SEC = 1'000'000;

  uint cppm_gpio;

  PIO pio;
  uint pio_offset;
  int pio_sm = -1;

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

  bool is_calibrating = false;
  double calibrating_min_us;
  double calibrating_max_us;

  bool initPIO();
  bool startDMA();
  double getChannelUs(uint ch);
};

#endif
