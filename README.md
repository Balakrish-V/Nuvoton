# Nuvoton
Basic Nuvoton Bare Metal Codes

the _millis function is developed to mimic the HAL_Tick function in the STM32.

_spi_0x00_0xff program just writes to check if the mosi line reacts to the change in code to visibly see the difference. when we send a constant digit, the oscilloscope is idle and the image is always the same.

the timer blocking function is simple delay logic using timer.

Also i might have pulled some functions from BSP examples. so verify the code with that also.

In the file w5500_ethernet_udp_interface, the code is being edited based on the c51 version. this include using differennt header file for stdint.h and stdbool.h as it is not available for this version of Keil.  C51 version doesn't support project file to be present very deep in file hierarchy. project file should be present with 3/4 levels of subdirectory. Proper include guards for all the headers. That much code optimization was not possible. Cannot initialize values based on binary digits. int k = 0b0001  <--- this is not allowed. Can't initialize variables like for(int i = 0; ...). Then cannot create variables wherever we like. WE should create the variables 1st and then only we should use then in our functions. Not like in STM32. There are some predefined keywords in Keil. We should not use them. 'data' , 'interrupt'  <---- don't create variables as such. So keeping all such considerations in mind, the library for interfacing nuvoton controller code is developed. (https://github.com/ussserrr/wiznet-library/tree/master) --> REFERENCE


There is some important theory in ethernet lib bug fixed file. Check it out. Also very valuable information about network related stacks. Thier tradeoffs and so on inside Network_STACKs file.

Comment out the following line in the recv_alloc function if you use them to receive instead of recv function.

  rx_start_ptr = SWAP_TWO_BYTES(rx_start_ptr);
    //rx_end_ptr = SWAP_TWO_BYTES(rx_end_ptr);  --> swapping these bytes cause flawed/corrupted/invalid tx/rx buffer pointer operations.

Note:
We have used the following Nuvoton Microcontrollers:-
1. MS51FB9AE (16Kb flash)
2. MS51EC0AE (32Kb flash)

The BSP is same for both the microcontrollers. But the memory mapping will vary obviously. Also there is pinout differences as the former is 20Pin IC and the latter is 28Pin IC. Check out for it. Also we have made some changes in main file and also in the separate driver file. So simply understanding the main alone will not provide sufficient information about the working of the code.
