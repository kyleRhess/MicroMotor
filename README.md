# MicroMotor

## A three-phase BLDC motor driver with a rotary encoder interface and serial communication via USB.
## [More project details can be found here.](https://kylerhess.github.io/umotor.html)
\
 This code has been written for and tested on an ARM STM32F410 processor.
\
\
The rotary encoder interface was originally developed in my [EncoderQ](https://github.com/kyleRhess/EncoderQ) project.
\
\
The only rotary encoder that has been tested so far is the [Omron E6D-C](http://www.ia.omron.com/data_pdf/cat/e6d-c_ds_e_5_1_csm497.pdf).
\
\
The only BLDC motor that has been tested so far is the [Maxon EC 339268](https://www.maxongroup.com/medias/sys_master/root/8841185394718/EN-282.pdf). The motor requirement is that it must include Hall sensor output for position sensing. Back-EMF control is not currently supported.