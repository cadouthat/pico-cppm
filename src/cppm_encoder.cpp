#include "pico_cppm/cppm_encoder.h"

#include <stdint.h>
#include <inttypes.h>

#include "pico/stdlib.h"
#include "hardware/dma.h"
#include "hardware/pio.h"

#include "pico_pio_loader/pico_pio_loader.h"

#include "cppm_encoder.pio.h"

bool CPPMEncoder::startOutput() {
  // Make sure we haven't already started
  if (dma_channel >= 0) {
    return false;
  }

  if (!startPIO()) {
    return false;
  }
  initDMABuffer();
  if (!startDMA()) {
    // Note: pico_pio_loader doesn't currently support removing programs
    // We can at least free up the state machine
    pio_sm_unclaim(pio, pio_sm);
    pio_sm = -1;
    return false;
  }
  return true;
}

void CPPMEncoder::setChannelValue(uint ch, double value) {
  assert(ch < NUM_CHANNELS);

  // Convert to duration
  double value_us = (value + 1) / 2 * (max_channel_us - min_channel_us) + min_channel_us;
  if (value_us > max_channel_us) {
    value_us = max_channel_us;
  }
  if (value_us < min_channel_us) {
    value_us = min_channel_us;
  }

  // Convert to loop count
  uint32_t value_count = value_us * clocks_per_us / cppm_encoder_CLOCKS_PER_COUNT;
  dma_buffer[ch] = value_count;
}

bool CPPMEncoder::startPIO() {
  pio_sm = pio_claim_unused_sm(pio, /*required=*/false);
  if (pio_sm < 0) {
    return false;
  }

  if (!pio_loader_add_or_get_offset(pio, &cppm_encoder_program, &pio_offset)) {
    pio_sm_unclaim(pio, pio_sm);
    pio_sm = -1;
    return false;
  }

  cppm_encoder_program_init(pio, pio_sm, pio_offset, cppm_gpio);

  // Unblock the PIO program by providing pulse duration via TX FIFO
  uint32_t pulse_count = pulse_us * clocks_per_us / cppm_encoder_CLOCKS_PER_COUNT;
  pio_sm_put_blocking(pio, pio_sm, pulse_count);
  return true;
}

void CPPMEncoder::initDMABuffer() {
  for (int i = 0; i < NUM_CHANNELS; i++) {
    setChannelValue(i, 0);
  }
  uint32_t sync_period_count = sync_period_us * clocks_per_us / cppm_encoder_CLOCKS_PER_COUNT;
  dma_buffer[NUM_CHANNELS] = sync_period_count;
}

bool CPPMEncoder::startDMA() {
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

  // This DMA channel will send the buffer to PIO TX FIFO
  dma_channel_config dma_config = dma_channel_get_default_config(dma_channel);
  channel_config_set_transfer_data_size(&dma_config, DMA_SIZE_32);
  channel_config_set_read_increment(&dma_config, true);
  channel_config_set_write_increment(&dma_config, false);
  channel_config_set_dreq(&dma_config, pio_get_dreq(pio, pio_sm, /*is_tx=*/true));
  channel_config_set_chain_to(&dma_config, dma_loop_channel);

  dma_channel_configure(
    dma_channel,
    &dma_config,
    &pio->txf[pio_sm],
    dma_buffer,
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
    &dma_channel_hw_addr(dma_channel)->read_addr,
    &dma_buffer_ptr,
    /*transfer_count=*/1,
    /*trigger=*/true);
  return true;
}
