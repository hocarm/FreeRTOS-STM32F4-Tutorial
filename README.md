# STM32F4-FreeRTOS-CubeMX

A demo project of FreeRTOS with CubeMX running on a STM32F4 Discovery board.
In this project STM32F4 interfacing with Gyroscope and USB

## Structure of this project
- F4-Gyro : Gyroscope with USB VCOM (USB-CDC)
- F4-Gyro-RTOS: Converting Gyroscope with USB VCOM example to RTOS based

## Steps to run this example

### Prerequisite

1. A PC running Windows.
2. A STM32F4Discovery board.
3. Keil uVision5.
4. USB Cable & other tools.

### Install the toolchain

The Keil uVision5 for ARM can be downloaded from its [website](http://www2.keil.com/mdk5). It's available for only Windows. For personal and education purpose, you can use MDK-Lite version with Code size restricted to 32 Kbyte.

### Install ST-Link utility

#### Windows
Grab the official utility from [ST website](http://www.st.com/web/catalog/tools/FM146/CL1984/SC724/SS1677/PF251168). Note that you should install the USB driver before install the st-util.

### Install STM32 Virtual COM Port Driver

#### Windows
Grab the official driver from [ST website](https://www.st.com/en/development-tools/stsw-stm32102.html)

### Compile this example
Open *.uvprojx on MDK-ARM folder and press F7 button.

### Debug
Connect your STM32F4Discovery with a USB cable. 
Press Ctrl + F5
Set breakpoint triggered at `main` function, and enjoy!
