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

int CPPMDecoder::dma_irq_index = -1;
CPPMDecoder* CPPMDecoder::dma_channel_to_instance[NUM_DMA_CHANNELS] = {0};

void CPPMDecoder::startListening() {
  // Make sure we haven't already started listening
  assert(dma_channel < 0);

  // Load and configure PIO program
  pio_offset = pio_add_program(pio, &cppm_decoder_program);
  pio_sm = pio_claim_unused_sm(pio, true);

  cppm_decoder_program_init(pio, pio_sm, pio_offset, cppm_gpio);

  // Configure a new DMA channel (interrupts are mapped to this instance)
  dma_channel = assignUnusedDMAChannel(this);
  dma_channel_config dma_config = dma_channel_get_default_config(dma_channel);
  channel_config_set_transfer_data_size(&dma_config, DMA_SIZE_32);
  channel_config_set_read_increment(&dma_config, false);
  channel_config_set_write_increment(&dma_config, true);
  channel_config_set_dreq(&dma_config, pio_get_dreq(pio, pio_sm, /*is_tx=*/false));

  // Fill our buffer from the PIO RX FIFO
  dma_channel_configure(
    dma_channel,
    &dma_config,
    dma_buffer,
    &pio->rxf[pio_sm],
    sizeof(dma_buffer) / sizeof(uint32_t),
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

void CPPMDecoder::sharedInit(uint dma_irq_index) {
  // If already initialized, do nothing
  if (CPPMDecoder::dma_irq_index >= 0) {
    return;
  }
  CPPMDecoder::dma_irq_index = dma_irq_index;

  uint irq_num = dma_irq_index ? DMA_IRQ_1 : DMA_IRQ_0;

  // Add our ISR as a shared handler, other handlers may exist for other DMA channels
  irq_add_shared_handler(irq_num, sharedISRForDMA, PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);
  irq_set_enabled(irq_num, true);
}

void CPPMDecoder::handleDMAFinished() {
  // Restart DMA transfer
  dma_channel_set_write_addr(dma_channel, dma_buffer, /*trigger=*/true);

  // FIXME
  //printf("DMA finished!\n");
}

uint CPPMDecoder::assignUnusedDMAChannel(CPPMDecoder* instance) {
  // sharedInit must be called first
  assert(dma_irq_index >= 0);

  uint channel = dma_claim_unused_channel(/*required=*/true);
  dma_irqn_set_channel_enabled(dma_irq_index, channel, true);

  dma_channel_to_instance[channel] = instance;

  return channel;
}

void CPPMDecoder::sharedISRForDMA() {
  // DMA ISR is shared for all channels, so we need to check each
  for (uint channel = 0; channel < NUM_DMA_CHANNELS; channel++) {
    // We only care about channels assigned to CPPMDecoder instances
    if (!dma_channel_to_instance[channel]) {
      continue;
    }

    // We only care about channels currently requesting an interrupt
    if (!dma_irqn_get_channel_status(dma_irq_index, channel)) {
      continue;
    }

    dma_irqn_acknowledge_channel(dma_irq_index, channel);

    dma_channel_to_instance[channel]->handleDMAFinished();
  }
}
