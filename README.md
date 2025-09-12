# Nuvoton
Basic Nuvoton Bare Metal Codes

the _millis function is developed to mimic the HAL_Tick function in the STM32.

_spi_0x00_0xff program just writes to check if the mosi line reacts to the change in code to visibly see the difference. when we send a constant digit, the oscilloscope is idle and the image is always the same.

the timer blocking function is simple delay logic using timer.

Also i might have pulled some functions from BSP examples. so verify the code with that also.
