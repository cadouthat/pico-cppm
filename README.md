# pico-cppm
cPPM library for the Raspberry Pi Pico using PIO + DMA

Supports continuously encoding and decoding cPPM signals on GPIOs without ever interrupting the CPU. Each encoder/decoder instance reserves 1 PIO state machine and 2 DMA channels.
