# Nordsense-exercise
This repository contains code for the board nRF52840-DK. It corresponds to a firmware code which blinks with four LEDs and connects to an
external RTC device. The project uses a LOGGER that through information about the state of the process on a virtual COM port.

In order to compile the system:

    make all

In order to flash the executable:

    make flash

In order to clean project:

    make clean

In order to check for the execution state:

    Windows: Install TeraTerm, open it and search for an available COM port; configure it at 115200 bauds

    Linux: Open a terminal and type for example 'cat /dev/ttyACM0 115200'
