# CMU JollyRoger STM32 firmware
This is the STM32 firmware repository for CMU mechatronics S2019 Team A JollyRoger project
ShipBot.

## Overview

This firmware has the following responsibilities:
- Sending TOF sensor readings to host PC
- Receive chassis commands from host PC
- Send chassis feedback to host PC

Communication will be established between host PC via HUART2 serial interface. Use a
USB-tty device to mount virtual COM interface and read/write to it to interact with
the STM32 controller.
