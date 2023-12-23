[![PlatformIO Registry](https://badges.registry.platformio.org/packages/dernasherbrezon/library/sx127x.svg)](https://registry.platformio.org/libraries/dernasherbrezon/sx127x) [![Component Registry](https://components.espressif.com/components/dernasherbrezon/sx127x/badge.svg)](https://components.espressif.com/components/dernasherbrezon/sx127x) [![Quality Gate Status](https://sonarcloud.io/api/project_badges/measure?project=dernasherbrezon_sx127x&metric=alert_status)](https://sonarcloud.io/summary/new_code?id=dernasherbrezon_sx127x) [![main](https://github.com/dernasherbrezon/sx127x/actions/workflows/main.yml/badge.svg)](https://github.com/dernasherbrezon/sx127x/actions/workflows/main.yml)

# About

Library to work with semtech sx127x chips.

# Features

There are several similar libraries exist, but this one is much better:

* LoRa, FSK and OOK modulations
* Support for resume from deep sleep. Most of libraries re-init the chip upon the startup and erase everything that was received previously. This library provides granular initialization functions. See ```examples/receive_lora_deepsleep/main/main.c``` for more info.
* Written in C. It is so much easier to integrate with another C project. But also possible to use from C++ project.
* Doesn't have external dependencies. This library is based on C99 standard.
* Can work with 2 or more modules connected to the same SPI bus.
* No busy loops for handling RX and TX events. See examples on how to configure and handle interrupts.
* Good documentation.
* Can be used on ESP32 or RaspberryPI or any other linux with GPIO pins.

This library supports all standard LoRa features:

* RX/TX
* CAD (Channel activity detection). Can be used to reduce power consumption in RX mode.
* TX with +20dbm power
* Explicit and implicit headers
* Granular sx127x register configuration

And FSK/OOK features:

* RX/TX
* Short messages and extra long messages (up to 2047 bytes). For messages more than 62 bytes digital pins DIO1 and DIO2 must be wired up and configured properly.
* CRC, Encoding, RSSI, address filtering, AFC and syncword configurations
* Fixed and variable packet formats
* Periodic beacons

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

## Custom architecture

It is possible to use this library in any other microcontroller architecture. To do this several steps are required. 

 1. Implement functions to work via SPI. Interface is defined in ```include/sx127x_spi.h``` and put implementation somewhere inside your project.
 2. Clone this library into your project
 3. Connect all things together in your application's cmake file:

```cmake
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/sx127x/include)
add_library(sx127x STATIC
        "${CMAKE_CURRENT_SOURCE_DIR}/sx127x/src/sx127x.c"
        "${CMAKE_CURRENT_SOURCE_DIR}/src/sx127x_custom_spi_implementation.c")
target_link_libraries(my_application sx127x)
```

## Functions

All functions follow the same format:

```
sx127x_(modulation)_(rx or tx or empty)_(set or get)_(parameter)
```

Where:

  * modulation - can be "lora", "fsk" or "ook" or "fsk_ook" or empty. If empty, then applicable for all modulation types
  * rx or tx - some functions specific for rx or tx. If empty, then applicable for both
  * set or get - normally functions operate over sx127x registers - either set or get them

# Examples

```examples``` folder contains the following examples:

* ```receive_lora``` - RX in LoRa mode and explicit header. Uses CAD to quickly detect presence of the message and switch into RX mode.
* ```receive_lora_deepsleep``` - RX in LoRa mode while in the deep sleep
* ```receive_lora_implicit_header``` - RX in LoRa mode and implicit header (without header)
* ```receive_lora_raspberrypi``` - RX in LoRa mode on RaspberryPI via GPIO pins and onboard SPI. Tested on module RA-02
* ```receive_fsk``` - RX in FSK mode. Variable packet length, NRZ encoding, CRC, AFC on.
* ```receive_fsk_filtered``` - RX in FSK mode where only messages with NODE address 0x11 and Broadcast NODE address 0x00 are accepted. Variable packet length, NRZ encoding, CRC, AFC on.
* ```receive_fsk_fixed``` - RX in FSK mode. Accepted packets with fixed packet length - 2047 bytes (max possible), NRZ encoding, CRC, AFC on.
* ```receive_fsk_raspberry``` - RX in FSK mode on RaspberryPI via GPIO pins and onboard SPI. Variable packet length, NRZ encoding, CRC, AFC on.
* ```receive_ook``` - RX in OOK mode. Variable packet length, NRZ encoding, CRC, AFC on.
* ```transmit_lora``` - TX in LoRa mode and explicit header. Several messages of different sizes and at all supported power levels
* ```transmit_lora_implicit_header``` - TX in LoRa mode and implicit header (without header)
* ```transmit_lora_raspberrypi``` - TX messages using LoRa mode from RaspberryPI. Tested on module RA-02
* ```transmit_fsk``` - TX in FSK mode. Variable packet length, NRZ encoding, CRC, AFC on. Sending several messages: small 2 byte, messages that can fit into FIFO fully, max messages for variable packet type - 255 bytes and same messages, but for node address 0x11 and 0x00.
* ```transmit_fsk_fixed``` - TX in FSK mode. Fixed packet length - 2047 bytes, NRZ encoding, CRC, AFC on.
* ```transmit_ook``` - TX in OOK mode. Variable packet length, NRZ encoding, CRC, AFC on. Sending several messages: small 2 byte, messages that can fit into FIFO fully, max messages for variable packet type - 255 bytes and same messages, but for node address 0x11 and 0x00.
* ```temperature``` - Constantly read raw temperature value from the internal sensor. Must be calibrated first using well-known temperature. Available in FSK mode.

# Tests

There are several unit tests in the ```test``` folder. They can be executed on any host machine using the commands below:

```
cd test
mkdir build
cd build
cmake ..
make test
```

## Integration tests

Integration tests can verify communication between real devices in different modes. Tests require two LoRa boards connected to the same host. It is possible to test on any other boards by overriding pin mappings in ```test/test_app/main.c```. By default tests assume transmitter and receiver is TTGO lora32.

Before running tests from ESP-IDF make sure [pytest is installed](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/contribute/esp-idf-tests-with-pytest.html).

Run the following command to test:

```
cd test/test_app
idf.py build
pytest --target esp32 --port="/dev/ttyACM0|/dev/ttyACM1" pytest_*
```
