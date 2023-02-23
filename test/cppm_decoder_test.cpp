#include <stdio.h>
#include <math.h>

#include "pico/stdlib.h"

#include "pico_cppm/cppm_decoder.h"

constexpr uint TEST_GPIO = 2;
constexpr double MIN_PERIOD_US = 1000;
constexpr double MAX_PERIOD_US = 2000;
constexpr double EXPECT_DELTA = 0.05;

void sendSync(uint pulse_us) {
  gpio_put(TEST_GPIO, true);
  sleep_us(pulse_us);
  gpio_put(TEST_GPIO, false);
  sleep_us(10000);
}

void sendPulses(const double durations[], uint pulse_us = 500, uint num_channels = 9, bool with_sync = true) {
  for (int i = 0; i < num_channels; i++) {
    uint32_t channel_us = MAX(0, (durations[i] + 1) / 2 * (MAX_PERIOD_US - MIN_PERIOD_US) + MIN_PERIOD_US);
    gpio_put(TEST_GPIO, true);
    sleep_us(pulse_us);
    gpio_put(TEST_GPIO, false);
    if (channel_us > pulse_us) {
      sleep_us(channel_us - pulse_us);
    }
  }
  if (with_sync) {
    sendSync(pulse_us);
  }
}

void expectChannels(CPPMDecoder& decoder, const double expected[], const char* test_name) {
  for (int i = 0; i < 9; i++) {
    double actual = decoder.getChannelValue(i);
    if (fabs(actual - expected[i]) > EXPECT_DELTA) {
      printf("Failure in \"%s\": expected %f; actual %f\n", test_name, expected[i], actual);
    }
  }
}

int main() {
  stdio_init_all();

  sleep_ms(2500);
  printf("Begin test\n");

  CPPMDecoder decoder(TEST_GPIO, pio0, 2500, 1000, 2000);
  CPPMDecoder::sharedInit(0);
  decoder.startListening();

  gpio_init(TEST_GPIO);
  gpio_put(TEST_GPIO, false);
  gpio_set_dir(TEST_GPIO, true);

  sendPulses((const double[]){0.75, 0.75, 0.75, 0.75, 0.75, 0.75, 0.75, 0.75, 0.75});
  expectChannels(decoder, (const double[]){0, 0, 0, 0, 0, 0, 0, 0, 0}, "discard first frame");

  sendPulses((const double[]){0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5});
  expectChannels(decoder, (const double[]){0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5}, "typical frame");

  sendPulses((const double[]){-0.99, 0.99, -0.99, 0.99, -0.99, 0.99, -0.99, 0.99, -0.99});
  expectChannels(decoder, (const double[]){-0.99, 0.99, -0.99, 0.99, -0.99, 0.99, -0.99, 0.99, -0.99}, "fast pulses");

  sendPulses((const double[]){0.99, 0.99, 0.99, 0.99, 0.99, 0.99, 0.99, 0.99, 0.99});
  expectChannels(decoder, (const double[]){0.99, 0.99, 0.99, 0.99, 0.99, 0.99, 0.99, 0.99, 0.99}, "slow pulses");

  sendPulses((const double[]){0.75, 0.75, 0.75}, 500, 3);
  expectChannels(decoder, (const double[]){0.75, 0.75, 0.75, 0, 0, 0, 0, 0, 0}, "too few channels");

  sendPulses((const double[]){0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.75, 0.75, 0.75}, 500, 12);
  expectChannels(decoder, (const double[]){0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5}, "too many channels");

  sendPulses((const double[]){-0.25}, 500, 1, false);
  // Rising edge
  gpio_put(TEST_GPIO, true);
  expectChannels(decoder, (const double[]){-0.25, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5}, "partial channels in realtime");
  sendPulses((const double[]){0}, 500, 1);

  sendPulses((const double[]){0.25}, 500, 1, false);
  gpio_put(TEST_GPIO, true);
  sleep_us(2500);
  expectChannels(decoder, (const double[]){0.25, 0, 0, 0, 0, 0, 0, 0, 0}, "long pulse error");
  gpio_put(TEST_GPIO, false);

  sendPulses((const double[]){0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5});
  expectChannels(decoder, (const double[]){0.25, 0, 0, 0, 0, 0, 0, 0, 0}, "discard first frame after error");

  sendPulses((const double[]){0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5});
  expectChannels(decoder, (const double[]){0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5}, "back to typical frame");

  printf("Test complete!");
  while(1) tight_loop_contents();

  return 0;
}
