# About

Library to work with LoRa chips sx127x.

# Features

There are several similar libraries exist, but this one is much better:

* Support for resume from deep sleep. Most of libraries re-init LoRa chip upon startup and erase everything that was received previously. This library provides granular initialization functions. See ```examples/receive_deepsleep/main/main.c``` for more info.
* Written in C. It is so much easier to integrate with another C project. But also possible to use from C++ project.
* Don't have external dependencies. This library based on native ESP32 APIs.
* Can work with 2 or more LoRa modules connected to the same SPI bus.

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

```
set(EXTRA_COMPONENT_DIRS /home/user/myprojects/esp/sx127x)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(my-esp-project)
```

## platformio

Add the following dependency to platformio.ini:

```
lib_deps = dernasherbrezon/sx127x
```

# Examples

```examples``` folder contains the following examples:

* ```receive``` - basic RX
* ```receive_deepsleep``` - receive messages from deep sleep
* ```transmit``` - basic TX

# Implementation notes

* Doesn't support FSK or OOK
