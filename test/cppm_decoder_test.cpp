#include <stdio.h>
#include <math.h>

#include "pico/stdlib.h"

#include "pico_cppm/cppm_decoder.h"

constexpr uint TEST_GPIO = 2;
constexpr uint SYNC_PERIOD_US = 20000;
constexpr uint DEFAULT_PULSE_US = 500;
constexpr double MIN_PERIOD_US = 1000;
constexpr double MAX_PERIOD_US = 2000;
constexpr double EXPECT_DELTA = 0.05;

constexpr bool PULSE_GPIO_STATE = false;

void sendSync(uint pulse_us) {
  gpio_put(TEST_GPIO, PULSE_GPIO_STATE);
  sleep_us(pulse_us);
  gpio_put(TEST_GPIO, !PULSE_GPIO_STATE);
  sleep_us(SYNC_PERIOD_US - pulse_us);
}

void sendPulses(const double durations[], uint pulse_us = 500, uint num_channels = 9, bool with_sync = true) {
  for (int i = 0; i < num_channels; i++) {
    uint32_t channel_us = MAX(0, (durations[i] + 1) / 2 * (MAX_PERIOD_US - MIN_PERIOD_US) + MIN_PERIOD_US);
    gpio_put(TEST_GPIO, PULSE_GPIO_STATE);
    sleep_us(pulse_us);
    gpio_put(TEST_GPIO, !PULSE_GPIO_STATE);
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

  CPPMDecoder decoder(TEST_GPIO, pio0, SYNC_PERIOD_US, MIN_PERIOD_US, MAX_PERIOD_US);
  decoder.startListening();

  gpio_init(TEST_GPIO);
  gpio_put(TEST_GPIO, !PULSE_GPIO_STATE);
  gpio_set_dir(TEST_GPIO, true);

  sendPulses((const double[]){0.75, 0.75, 0.75, 0.75, 0.75, 0.75, 0.75, 0.75, 0.75});
  sleep_ms(1);
  expectChannels(decoder, (const double[]){0, 0, 0, 0, 0, 0, 0, 0, 0}, "discard first frame");

  sendPulses((const double[]){-1, 1, -1, 1, -1, 1, -1, 1, -1});
  sleep_ms(1);
  expectChannels(decoder, (const double[]){-1, 1, -1, 1, -1, 1, -1, 1, -1}, "switching polarity");

  sendPulses((const double[]){1, 1, -1, -1, 1, 1, -1, -1, 1}, DEFAULT_PULSE_US, 9, false);
  gpio_put(TEST_GPIO, PULSE_GPIO_STATE);
  sleep_us(DEFAULT_PULSE_US);
  gpio_put(TEST_GPIO, !PULSE_GPIO_STATE);
  sleep_us(SYNC_PERIOD_US * 10);
  expectChannels(decoder, (const double[]){1, 1, -1, -1, 1, 1, -1, -1, 1}, "long sync period");

  sendPulses((const double[]){-1, 1, -1, 1, -1, 1, -1, 1, -1}, 950);
  sleep_ms(1);
  expectChannels(decoder, (const double[]){-1, 1, -1, 1, -1, 1, -1, 1, -1}, "slow pulses");

  sendPulses((const double[]){1, 1, -1, -1, 1, 1, -1, -1, 1}, 50);
  sleep_ms(1);
  expectChannels(decoder, (const double[]){1, 1, -1, -1, 1, 1, -1, -1, 1}, "fast pulses");

  sendPulses((const double[]){0.75, 0.75, 0.75}, DEFAULT_PULSE_US, 3);
  sleep_ms(1);
  expectChannels(decoder, (const double[]){0.75, 0.75, 0.75, 0, 0, 0, 0, 0, 0}, "too few channels");

  sendPulses((const double[]){0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.75, 0.75, 0.75}, DEFAULT_PULSE_US, 12);
  sleep_ms(1);
  expectChannels(decoder, (const double[]){0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5}, "too many channels");

  sendPulses((const double[]){-0.25}, DEFAULT_PULSE_US, 1, false);
  // Rising edge
  gpio_put(TEST_GPIO, PULSE_GPIO_STATE);
  sleep_ms(1);
  expectChannels(decoder, (const double[]){-0.25, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5}, "partial channels in realtime");
  sendPulses((const double[]){0}, DEFAULT_PULSE_US, 1);

  sendPulses((const double[]){0.25}, DEFAULT_PULSE_US, 1, false);
  gpio_put(TEST_GPIO, PULSE_GPIO_STATE);
  sleep_us(SYNC_PERIOD_US);
  sleep_ms(1);
  expectChannels(decoder, (const double[]){0.25, 0, 0, 0, 0, 0, 0, 0, 0}, "long pulse error");
  gpio_put(TEST_GPIO, !PULSE_GPIO_STATE);

  sendPulses((const double[]){0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5});
  sleep_ms(1);
  expectChannels(decoder, (const double[]){0.25, 0, 0, 0, 0, 0, 0, 0, 0}, "discard first frame after error");

  double expected[9];
  for (float v = -1; v < 1; v += 0.05) {
    for (int ch = 0; ch < 9; ch++) {
      expected[ch] = v;
    }
    sendPulses(expected);
    sleep_ms(1);
    expectChannels(decoder, expected, "rapidly changing");
  }

  printf("Test complete!");
  while(1) tight_loop_contents();

  return 0;
}
