#include "pico_cppm/cppm_decoder.h"

#include <stdint.h>
#include <inttypes.h>

#include "pico/stdlib.h"
#include "hardware/dma.h"
#include "hardware/pio.h"

#include "pico_pio_loader/pico_pio_loader.h"

#include "cppm_decoder.pio.h"

bool CPPMDecoder::startListening() {
  // Make sure we haven't already started
  if (dma_channel >= 0) {
    return false;
  }

  if (!initPIO()) {
    return false;
  }
  if (!startDMA()) {
    // Note: pico_pio_loader doesn't currently support removing programs
    // We can at least free up the state machine
    pio_sm_unclaim(pio, pio_sm);
    pio_sm = -1;
    return false;
  }

  // Unblock the PIO program by providing maximum period count via TX FIFO
  max_period_count = max_period_us * clocks_per_us / cppm_decoder_CLOCKS_PER_COUNT;
  pio_sm_put_blocking(pio, pio_sm, max_period_count);
  return true;
}

double CPPMDecoder::getChannelValue(uint ch) {
  double last_us = getChannelUs(ch);

  // 0 indicates that a channel value is not available
  if (!last_us) {
    return 0;
  }

  // Use calibration to convert duration to [-1, 1] value range
  double p = (last_us - calibrated_min_us) / (calibrated_max_us - calibrated_min_us);
  p = p * 2 - 1;

  if (p < -1) {
    return -1;
  }
  if (p > 1) {
    return 1;
  }
  return p;
}

void CPPMDecoder::beginCalibration() {
  is_calibrating = true;
  calibrating_min_us = getChannelUs(0);
  calibrating_max_us = getChannelUs(0);
}
void CPPMDecoder::processCalibration() {
  if (!is_calibrating) {
    return;
  }

  for (uint ch = 0; ch < cppm_decoder_NUM_CHANNELS; ch++) {
    double value = getChannelUs(ch);
    calibrating_min_us = MIN(calibrating_min_us, value);
    calibrating_max_us = MAX(calibrating_max_us, value);
  }
}
bool CPPMDecoder::endCalibration(double min_spread_us) {
  if (!is_calibrating) {
    return false;
  }
  is_calibrating = false;

  if (calibrating_max_us - calibrating_min_us < min_spread_us) {
    return false;
  }

  calibrated_min_us = calibrating_min_us;
  calibrated_max_us = calibrating_max_us;
  return true;
}

bool CPPMDecoder::initPIO() {
  pio_sm = pio_claim_unused_sm(pio, /*required=*/false);
  if (pio_sm < 0) {
    return false;
  }

  if (!pio_loader_add_or_get_offset(pio, &cppm_decoder_program, &pio_offset)) {
    pio_sm_unclaim(pio, pio_sm);
    pio_sm = -1;
    return false;
  }

  // Enable the state machine, but it will be blocked waiting for configuration
  cppm_decoder_program_init(pio, pio_sm, pio_offset, cppm_gpio);
  return true;
}

bool CPPMDecoder::startDMA() {
  dma_channel = dma_claim_unused_channel(/*required=*/false);
  if (dma_channel < 0) {
    return false;
  }
  dma_loop_channel = dma_claim_unused_channel(/*required=*/false);
  if (dma_loop_channel < 0) {
    dma_channel_unclaim(dma_channel);
    dma_channel = -1;
    return false;
  }

  // This DMA channel will fill the buffer from the PIO RX FIFO
  dma_channel_config dma_config = dma_channel_get_default_config(dma_channel);
  channel_config_set_transfer_data_size(&dma_config, DMA_SIZE_32);
  channel_config_set_read_increment(&dma_config, false);
  channel_config_set_write_increment(&dma_config, true);
  channel_config_set_dreq(&dma_config, pio_get_dreq(pio, pio_sm, /*is_tx=*/false));
  channel_config_set_chain_to(&dma_config, dma_loop_channel);

  dma_channel_configure(
    dma_channel,
    &dma_config,
    dma_buffer,
    &pio->rxf[pio_sm],
    sizeof(dma_buffer) / sizeof(uint32_t),
    /*trigger=*/false);

  // This DMA channel will reset the first DMA channel to loop forever
  dma_channel_config dma_loop_config = dma_channel_get_default_config(dma_loop_channel);
  channel_config_set_transfer_data_size(&dma_loop_config, DMA_SIZE_32);
  channel_config_set_read_increment(&dma_loop_config, false);
  channel_config_set_write_increment(&dma_loop_config, false);
  channel_config_set_chain_to(&dma_loop_config, dma_channel);

  dma_channel_configure(
    dma_loop_channel,
    &dma_loop_config,
    &dma_channel_hw_addr(dma_channel)->write_addr,
    &dma_buffer_ptr,
    /*transfer_count=*/1,
    /*trigger=*/true);
  return true;
}

double CPPMDecoder::getChannelUs(uint ch) {
  if (ch >= cppm_decoder_NUM_CHANNELS) {
    return 0;
  }
  // 0 indicates that a channel value is not available
  if (!dma_buffer[ch]) {
    return 0;
  }

  // PIO deducts from max period count and pushes the number remaining
  uint32_t last_count = max_period_count - dma_buffer[ch];

  return (last_count / (double)clocks_per_us) * cppm_decoder_CLOCKS_PER_COUNT;
}
