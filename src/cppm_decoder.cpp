#include "pico_cppm/cppm_decoder.h"

#include <stdint.h>
#include <stdio.h>
#include <inttypes.h>

#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/pio.h"

#include "cppm_decoder.pio.h"

namespace {

constexpr uint32_t MICROS_PER_SEC = 1'000'000;

} // namespace

void CPPMDecoder::startListening() {
  // Make sure we haven't already started listening
  assert(dma_channel < 0);

  // Load and configure PIO program
  pio_offset = pio_add_program(pio, &cppm_decoder_program);
  pio_sm = pio_claim_unused_sm(pio, true);

  cppm_decoder_program_init(pio, pio_sm, pio_offset, cppm_gpio);

  dma_channel = dma_claim_unused_channel(/*required=*/true);
  dma_loop_channel = dma_claim_unused_channel(/*required=*/true);

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

  // Unblock the PIO program by providing maximum period count via TX FIFO
  clocks_per_us = clock_get_hz(clk_sys) / MICROS_PER_SEC;
  max_period_count = max_period_us * clocks_per_us / cppm_decoder_CLOCKS_PER_COUNT;
  pio_sm_put_blocking(pio, pio_sm, max_period_count);
}

double CPPMDecoder::getChannelValue(uint ch) {
  if (ch >= cppm_decoder_NUM_CHANNELS) {
    return 0;
  }
  // 0 indicates that a channel value is not available
  if (!dma_buffer[ch]) {
    return 0;
  }

  // PIO deducts from max period count and pushes the number remaining
  uint32_t last_count = max_period_count - dma_buffer[ch];

  double last_us = (last_count / (double)clocks_per_us) * cppm_decoder_CLOCKS_PER_COUNT;

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
