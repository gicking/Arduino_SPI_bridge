# Overview

_Arduino_SPI_bridge_ is a small Arduino project to configure an Arduino as USB<->SPI gateway. Basically it is remotely controlled via USB and a simple UART protocol (for details see 'protocol.ods'). So far it has only been tested with an Arduino UNO R3, but there should be no major issue on other ATMega boards. However, the ARM-based Arduino boards are not yet supported, since this project requires the [NeoHWSerial](https://github.com/SlashDevin/NeoHWSerial) library, which so far only runs on the older ATMega-based boards. 

To test the functionality, a small Python project is also provided. It is compatible with Windows, MacOS X and Linux, including Raspbian, provided python and pySerial are installed.

Note: the main objective for this project was to provide an OS-independent SPI-device for the [_stm8gal_](https://github.com/gicking/stm8gal) application. **Unfortunately the combination of _stm8gal_ and _Arduino_SPI_bridge_ doesn't seem to work, yet**

For bug reports or feature requests please send me a note.

Have fun!
Georg

***

# License / Disclaimer
- _Arduino_SPI_bridge_ and it's source code is distributed under the Apache License Version 2.0 (see [License](LICENSE))

***

# Building the Software

Open 'SPI_bridge/SPI_bridge.ino' in the Arduino IDE, build and upload to the Arduino board. If required, install [NeoHWSerial](https://github.com/SlashDevin/NeoHWSerial) first.

***

# Using the Software

_Arduino_SPI_bridge_ waits for commands via USB and reacts accordingly. Frame syncronization is via inter-frame time (>1ms). The simple command and response protocol is (also see 'protocol.ods') 

<p align="center"> 
  <img src="images/protocol.png">
</p>

An example of protocol and possible error handling is given in 'Arduino_SPI_bridge.py'. This file can also be imported into bigger Python programs.

***

# General Notes

- before connecting different devices, please assert compatible voltage levels. Specifically, to avoid damage **never expose a 3.3V device to 5V signals**
  
***

# Known Issues / Limitations

- the combination of _stm8gal_ and _Arduino_SPI_bridge_ doesn't work yet.

***

# Revision History

v1.0.0 (2017-12-21)
  - initial release by Georg Icking-Konert under the Apache License 2.0

