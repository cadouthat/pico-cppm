#ifndef __HARDWARE_RR12RDTS_H__
#define __HARDWARE_RR12RDTS_H__

#include <stdint.h>

#include "hardware/dma.h"
#include "hardware/pio.h"

#include "cppm_decoder.pio.h"

namespace {

constexpr uint32_t DEFAULT_MAX_PERIOD_US = 2500;
constexpr double DEFAULT_CALIBRATED_MIN_US = 900;
constexpr double DEFAULT_CALIBRATED_MAX_US = 2200;

} // namespace

class CPPMDecoder {
 public:
  CPPMDecoder(uint cppm_gpio, PIO pio = pio0,
    uint32_t max_period_us = DEFAULT_MAX_PERIOD_US,
    double calibrated_min_us = DEFAULT_CALIBRATED_MIN_US,
    double calibrated_max_us = DEFAULT_CALIBRATED_MAX_US)
    : cppm_gpio(cppm_gpio), pio(pio),
    max_period_us(max_period_us),
    calibrated_min_us(calibrated_min_us),
    calibrated_max_us(calibrated_max_us) {}
  // TODO: cleanup in destructor

  // Start PIO/DMA, which will begin populating channel values via interrupts
  void startListening();

  // Get the latest value for channel index ch, in range [-1, 1]
  double getChannelValue(uint ch);

  // TODO: calibration mode

  // Global setup routine for CPPMDecoder interrupts. This must have been called at least once
  // before startListening()
  // dma_irq_index must be 0 or 1, corresponding to DMA_IRQ_0 or DMA_IRQ_1
  static void sharedInit(uint dma_irq_index);

 private:

  uint cppm_gpio;

  PIO pio;
  uint pio_offset = 0;
  uint pio_sm = 0;

  int dma_channel = -1;
  volatile uint32_t dma_buffer[cppm_decoder_NUM_CHANNELS] = {0};

  uint32_t clocks_per_us = 0;
  uint32_t max_period_count = 0;
  // Maximum pulse-to-pulse period before starting a new frame
  uint32_t max_period_us;
  // Channel period that maps to -1
  double calibrated_min_us;
  // Channel period that maps to 1
  double calibrated_max_us;

  static int dma_irq_index;
  // Map from DMA channel to CPPMDecoder instance which claims it (if any)
  static CPPMDecoder* dma_channel_to_instance[NUM_DMA_CHANNELS];

  void handleDMAFinished();

  static uint assignUnusedDMAChannel(CPPMDecoder* instance);
  static void sharedISRForDMA();
};

#endif
