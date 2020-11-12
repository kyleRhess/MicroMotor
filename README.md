# EncoderQ

## A realtime embedded C program for interfacing with incremental rotary encoders

 This code has been written for and tested on an ARM STM32F4 processor with a timer peripheral that supports quadrature encoder input.
\
\
The processor hardware from the [DroneFluxx](https://github.com/kyleRhess/DroneFluxx) project was initially used for prototyping.
\
The only rotary encoder that has been tested so far is the [E6D-C](http://www.ia.omron.com/data_pdf/cat/e6d-c_ds_e_5_1_csm497.pdf) from Omron. This is a 6,000 pulse/revolution incremental encoder. This encoder is capable of quadrature operation which essentially means it can output up to (4*6,000) = 24,000 pulses/revolution.
