#ifndef __PICO_CPPM_CPPM_ENCODER_H__
#define __PICO_CPPM_CPPM_ENCODER_H__

#include <stdint.h>

#include "hardware/clocks.h"
#include "hardware/pio.h"

class CPPMEncoder {
 public:
  CPPMEncoder(uint cppm_gpio, PIO pio = pio0,
    double pulse_us = DEFAULT_PULSE_US,
    double min_channel_us = DEFAULT_MIN_CHANNEL_US,
    double max_channel_us = DEFAULT_MAX_CHANNEL_US,
    double sync_period_us = DEFAULT_SYNC_PERIOD_US)
    : cppm_gpio(cppm_gpio), pio(pio),
    pulse_us(pulse_us),
    min_channel_us(min_channel_us),
    max_channel_us(max_channel_us),
    sync_period_us(sync_period_us) {
    clocks_per_us = clock_get_hz(clk_sys) / MICROS_PER_SEC;
  }
  // TODO: cleanup in destructor

  // Start PIO/DMA, which will begin generating cPPM output continuously
  bool startOutput();

  // Set the value for channel index ch, in range [-1, 1]
  void setChannelValue(uint ch, double value);

 private:
  static constexpr uint32_t MICROS_PER_SEC = 1'000'000;

  static constexpr uint NUM_CHANNELS = 9;

  static constexpr double DEFAULT_PULSE_US = 500;
  static constexpr double DEFAULT_MIN_CHANNEL_US = 1000;
  static constexpr double DEFAULT_MAX_CHANNEL_US = 2000;
  static constexpr double DEFAULT_SYNC_PERIOD_US = 5000;

  uint cppm_gpio;

  PIO pio;
  uint pio_offset;
  int pio_sm = -1;

  int dma_channel = -1;
  // Leave room for one extra entry to store sync pulse
  volatile uint32_t dma_buffer[NUM_CHANNELS + 1] = {0};
  int dma_loop_channel = -1;
  volatile uint32_t* dma_buffer_ptr = (uint32_t*)&dma_buffer;

  double pulse_us;
  double min_channel_us;
  double max_channel_us;
  double sync_period_us;

  uint32_t clocks_per_us = 0;

  bool startPIO();
  void initDMABuffer();
  bool startDMA();
};

#endif
