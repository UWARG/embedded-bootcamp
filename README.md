# Embedded-Bootcamp

## Welcome to the WARG Firmware Bootcamp! 

Full instructions can be found on the [Bootcamp Confluence Page](https://uwarg-docs.atlassian.net/wiki/spaces/BOOT/pages/1997373445/2021+Firmware+Bootcamp).

This repository contains a basic STM32IDE project that will be modified to complete the bootcamp.

For git specific questions and direction please visit our [Git Tutorial](https://uwarg-docs.atlassian.net/wiki/spaces/TUT/pages/1544257554/Git+and+GitHub+Tutorial).

---

# My Notes
- Input: potentiometer value (range: $[0, 3.3] \text{ volts}$.)
- Task: convert input to PWM signal to control a motor.
	- PWM signal (pulse width modulation) has something to do with modulating the duration, direction of the pulse (voltage).
- Potentiometer is connected to ADC chip (analog-to-digital converter chip).
- ADC sends this data to MCU (microcontroller unit).
- The data is transferred using SPI (serial peripheral interface)
	- SPI is a protocol for comms between ICs.
- The MCU receives digital input from the ADC and generates a PWM signal.

- Basically we plug in a motor to our board and test if it works and get the relevant parameters.

## Tools
- cmake (via homebrew)
- GNU Arm Toolchain (not GNU ARM Embedded Toolchain that's outdated) `gcc-arm-embedded` brew bottle.
- STM32CubeMX
	- Download binary
	- Double-click doesn't work => run installer using latest java version (install it's cask `temurin`).
	- Run actual app using java too.
## Configuring Pins
- MCU connects to other components using pins.
- To tell code which pin is which component, we use an IOC file.

## Code gen
- We use generated code for configuring prescalers, timers, stuff like that.
- While adding our code to the generated code, make sure you add between the comments. Otherwise the next generation will overwite it.

## Actual coding
tmrw
