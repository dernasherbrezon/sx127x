[![PlatformIO Registry](https://badges.registry.platformio.org/packages/dernasherbrezon/library/sx127x.svg)](https://registry.platformio.org/libraries/dernasherbrezon/sx127x) [![Component Registry](https://components.espressif.com/components/dernasherbrezon/sx127x/badge.svg)](https://components.espressif.com/components/dernasherbrezon/sx127x) [![Quality Gate Status](https://sonarcloud.io/api/project_badges/measure?project=dernasherbrezon_sx127x&metric=alert_status)](https://sonarcloud.io/summary/new_code?id=dernasherbrezon_sx127x) [![main](https://github.com/dernasherbrezon/sx127x/actions/workflows/main.yml/badge.svg)](https://github.com/dernasherbrezon/sx127x/actions/workflows/main.yml)

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
* CAD (Channel activity detection). Can be used to reduce power consumption in RX mode.
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

## Functions

All functions follow the same format:

```
sx127x_(modulation)_(rx or tx or empty)_(set or get)_(parameter)
```

Where:

  * modulation - can be "fsk" or "ook" or "fsk_ook" or empty. If empty, then applicable for all modulation types
  * rx or tx - some functions specific for rx or tx. If empty, then applicable for both
  * set or get - normally functions operate over sx127x registers - either set or get them

# Examples

```examples``` folder contains the following examples:

* ```receive_lora``` - RX in LoRa mode and explicit header
* ```receive_lora_deepsleep``` - RX in LoRa mode while in the deep sleep
* ```receive_lora_implicit_header``` - RX in LoRa mode and implicit header (without header)
* ```receive_lora_raspberrypi``` - RX in LoRa mode on RaspberryPI via GPIO pins and onboard SPI. Tested on module RA-02
* ```transmit_lora``` - TX in LoRa mode and explicit header. Several messages of the different size and at all supported power levels
* ```transmit_lora_implicit_header``` - TX in LoRa mode and implicit header (without header)
* ```transmit_lora_raspberrypi``` - TX messages using LoRa mode from RaspberryPI. Tested on module RA-02

# Implementation notes

* Doesn't support FSK or OOK
