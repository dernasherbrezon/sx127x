[![PlatformIO Registry](https://badges.registry.platformio.org/packages/dernasherbrezon/library/sx127x.svg)](https://registry.platformio.org/libraries/dernasherbrezon/sx127x)

# About

Library to work with LoRa chips sx127x.

# Features

There are several similar libraries exist, but this one is much better:

* Support for resume from deep sleep. Most of libraries re-init LoRa chip upon startup and erase everything that was received previously. This library provides granular initialization functions. See ```examples/receive_deepsleep/main/main.c``` for more info.
* Written in C. It is so much easier to integrate with another C project. But also possible to use from C++ project.
* Doesn't have external dependencies. This library is based on native ESP32 APIs.
* Can work with 2 or more LoRa modules connected to the same SPI bus.
* No busy loops for handling RX and TX events. See examples on how to configure and handle interrupts.
* Good documentation.
* Can be used on ESP32 or RaspberryPI or any other linux with GPIO pins.

And of course this library supports all standard LoRa features:

* RX/TX
* TX with +20dbm power
* Explicit and implicit headers
* Granular sx127x register configuration

# How to use

## esp-idf

Clone this repository somewhere, e.g.:

```
cd ~/myprojects/esp
git clone https://github.com/dernasherbrezon/sx127x.git
```

Add path to components:

```cmake
set(EXTRA_COMPONENT_DIRS /home/user/myprojects/esp/sx127x)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(my-esp-project)
```

## platformio

Add the following dependency to platformio.ini:

```
lib_deps = dernasherbrezon/sx127x
```

## Linux

Linux version depends on kernel SPI driver only. Make sure it is enabled. Clone this repository somewhere, e.g.:

```
cd ~/myprojects/esp
git clone https://github.com/dernasherbrezon/sx127x.git
```

Add subdirectory:

```cmake
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/sx127x)
target_link_libraries(my_application sx127x)
```

# Examples

```examples``` folder contains the following examples:

* ```receive``` - basic RX
* ```receive_deepsleep``` - receive messages from deep sleep
* ```receive_implicit_header``` - receive messages without header (implicit header)
* ```receive_raspberrypi``` - sample program to receive messages on RaspberryPI via GPIO pins and onboard SPI. Tested on module RA-02.  
* ```transmit``` - basic TX
* ```transmit_different_power``` - TX on different power levels
* ```transmit_implicit_header``` - transmit messages without header (implicit header)

# Implementation notes

* Doesn't support FSK or OOK
