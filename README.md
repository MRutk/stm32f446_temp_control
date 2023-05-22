# stm32f446_temp_control

Project for my engineerign degree thesis.
Using a stm32 nucleo f446 with freeRTOS and ds18b20 temp. sensor to control temperature of a substance by switching a relay powering a heat source.
Implemented a PI controller
Temperature, system status is displayed on a oled screen communicating with i2c.
Ds18b20 communicates with one-wire.
4 physical buttons allow to change desired target temperature, PI variables and manualy operate heating device.
Implemented three tasks: pulling temperature reading, updating screen and heat control/PID.

