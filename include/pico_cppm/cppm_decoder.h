#ifndef __HARDWARE_RR12RDTS_H__
#define __HARDWARE_RR12RDTS_H__

#include <stdint.h>

#include "hardware/dma.h"
#include "hardware/pio.h"

class CPPMDecoder {
 public:
  CPPMDecoder(uint cppm_gpio, PIO pio = pio0)
    : cppm_gpio(cppm_gpio), pio(pio) {}
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
  static constexpr uint CPPM_CHANNELS = 9;
  static constexpr uint DMA_TRANSFER_SIZE = CPPM_CHANNELS + 1;

  static constexpr double DEFAULT_MIN_PULSE_US = 100;
  static constexpr double DEFAULT_SYNC_THRESHOLD_US = 2500;
  static constexpr double DEFAULT_CALIBRATED_MIN_US = 900;
  static constexpr double DEFAULT_CALIBRATED_MAX_US = 2200;

  uint cppm_gpio;

  PIO pio;
  uint pio_offset = 0;
  uint pio_sm = 0;

  uint32_t clocks_per_us = 0;

  int dma_channel = -1;
  uint32_t dma_buffer[DMA_TRANSFER_SIZE] = {0};

  // Last known values for each channel
  double last_channel_us[CPPM_CHANNELS] = {0};
  uint ch_write_index = CPPM_CHANNELS;

  // Minimum "high" IO duration (cumulative) to be considered a pulse
  const double min_pulse_us = DEFAULT_MIN_PULSE_US;
  // Maximum pulse-to-pulse period to be a channel value, instead of a sync
  double sync_threshold_us = DEFAULT_SYNC_THRESHOLD_US;
  // Channel value that maps to -1
  double calibrated_min_us = DEFAULT_CALIBRATED_MIN_US;
  // Channel value that maps to 1
  double calibrated_max_us = DEFAULT_CALIBRATED_MAX_US;

  static int dma_irq_index;
  // Map from DMA channel to CPPMDecoder instance which claims it (if any)
  static CPPMDecoder* dma_channel_to_instance[NUM_DMA_CHANNELS];

  void handleDMAFinished();

  static uint assignUnusedDMAChannel(CPPMDecoder* instance);
  static void sharedISRForDMA();
};

#endif
