# pico-cppm
cPPM library for the Raspberry Pi Pico using PIO + DMA

Supports continuously encoding and decoding cPPM signals on GPIOs without ever interrupting the CPU. Each encoder/decoder instance reserves 1 PIO state machine and 2 DMA channels.

# cPPM Protocol Summary

Each channel in a frame is represented by a fixed duration LOW output ("pulse"), followed by a variable period of HIGH output. The channel value is proportional to the period between the falling (starting) edges of two consecutive pulses. The final channel in each frame is followed by another LOW pulse, and a significantly longer period of HIGH output ("sync period").
