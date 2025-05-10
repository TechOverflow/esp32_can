esp32_can (twai_v2)
==========

This fork is updated for dual TWAI controllers (ESP32-C6) and adds relevant new examples.

Credits:<br>
[Collin80/esp32_can](https://github.com/collin80/esp32_can) - Original esp32_can library<br>
[outlandnish/esp32_can](https://github.com/outlandnish/esp32_can) - Majority of twai_v2 library updates<br>
[sans-ltd/esp32_can](https://github.com/sans-ltd/esp32_can) - Bugfixes

Implements a CAN driver for the built-in CAN hardware on an ESP32. 
The builtin CAN is called CAN0, and also CAN1 if there is a second CAN port on the ESP32 chip in use.
This library requires the can_common library. That library is a common base that 
other libraries can be built off of to allow a more universal API for CAN.

The needed can_common library is found here: https://github.com/collin80/can_common
Some examples use the SmartLeds library found here: https://github.com/RoboticsBrno/SmartLeds
