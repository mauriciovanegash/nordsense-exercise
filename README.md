# Nordsense-exercise
This repository contains code for the board nRF52840-DK. It corresponds to a firmware code which blinks with four LEDs and connects to an
external RTC device. The project uses a LOGGER that throught information about the state of the process on a virtual COM port.
The setup for this repository has been defined to be used in Linux.

System setup (Linux):
    Download SDK for board nRF52840-DK
    Install gcc-toolchain version 'gcc-arm-none-eabi-6-2017-q2-update'
    Install program 'nrfjprog'
    Install JLink
    Clone this repository and change the variable $(SDK_ROOT) within Makefile with the path to the SDK for board nRF52840-DK

In order to compile the system:

    make all

In order to flash the executable:

    make flash

In order to clean project:

    make clean

In order to check for the execution state:

    Open a terminal and type for example 'cat /dev/ttyACM0 115200'
