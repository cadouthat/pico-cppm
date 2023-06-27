# pico-cppm
cPPM library for the Raspberry Pi Pico using PIO + DMA

Supports encoding and decoding cPPM signals on GPIOs. Encoding can be done continuously without ever interrupting the CPU. Decoding requires an interrupt after each cPPM frame to verify the number of channels decoded.

Each encoder instance requires 1 PIO state machine and 2 DMA channels. Each decoder instance requires 1 PIO state machine and 1 DMA channel. Encoders and decoders both require sufficient PIO program space, which is re-used across encoder and decoder instances respectively.

# cPPM Protocol Summary

Each channel in a frame is represented by a fixed duration LOW output ("pulse"), followed by a variable period of HIGH output. The channel value is proportional to the period between the falling (starting) edges of two consecutive pulses. The final channel in each frame is followed by another LOW pulse, and a significantly longer period of HIGH output ("sync period").
