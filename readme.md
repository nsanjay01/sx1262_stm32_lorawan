# LoRaWAN End Node Readme

---

## Overview

This is a simple demo application for a LoRa modem connecting to a LoRa network server (e.g., Loriot). The transmitted data can be monitored on the network server. Communication traces are displayed via UART.

## Description

This directory contains source files implementing a demo of a LoRaWAN end device, also called a LoRa Object. The device can be:

- A NUCLEO-F446RE board with LoRa Radio expansion board, optionally with a sensor board.

By configuring the LoRa IDs in the `se-identity.h` file according to the network requirements, the device periodically sends sensor data to the LoRa network.

## Keywords

Applications, SubGHz_Phy, LoRaWAN, End_Node

## Directory Contents

### Core Include Files

- `adc.h` — Prototypes for adc.c
- `adc_if.h` — ADC interface configuration
- `dma.h` — Prototypes for dma.c
- `main.h` — Common application defines
- `platform.h` — General hardware instances configuration
- `rtc.h` — Prototypes for rtc.c
- `rtc_if.h` — RTC interface configuration
- `stm32l4xx_hal_conf.h` — HAL configuration
- `stm32l4xx_it.h` — Interrupt handlers
- `stm32_lpm_if.h` — Low Power Manager interface config
- `sys_app.h` — Prototypes for system application
- `sys_conf.h` — Application configuration (debug, trace, low power, sensors)
- `sys_debug.h` — Debug configuration
- `sys_sensors.h` — Sensors application header
- `usart.h` — Prototypes for usart.c
- `usart_if.h` — USART interface configuration
- `utilities_conf.h` — Configuration for utilities
- `utilities_def.h` — Definitions for utility modules

### LoRaWAN Application Files

- `app_lorawan.h` — LRWAN middleware application header
- `CayenneLpp.h` — Implements Cayenne Low Power Protocol
- `Commissioning.h` — End-device commissioning parameters
- `lora_app.h` — LRWAN middleware application header
- `lora_app_version.h` — Application version definition
- `lora_info.h` — LoRaWAN configuration information
- `se-identity.h` — Secure element identity and keys

### Target Configuration Files

- `iks01a2_conf.h` & `iks01a3_conf.h` — MEMS component bus interface definitions
- `lorawan_conf.h` — LoRaWAN middleware configuration
- `mw_log_conf.h` — Trace configuration (enable/disable)
- `nucleo_l476rg_bus.h` — BSP BUS IO driver header
- `nucleo_l476rg_errno.h` — Error codes
- `radio_board_if.h` — Radio interface configuration
- `radio_conf.h` — Radio configuration header
- Various `..._conf.h` files for pin mappings and shield configuration

### Source Files

- ADC, DMA, RTC, USART configurations and handlers
- Main program body (`main.c`)
- Interrupt service routines
- Low power management functions
- System and debug utilities
- LoRaWAN middleware application source files

## Supported Hardware and Environment

- STM32L053R8, STM32L073RZ, STM32L152RE, STM32L476RG devices.
- Tested on STMicroelectronics Nucleo-F446RE boards.
- Easily adaptable to other supported devices and development boards.

### Setup Instructions

- Connect your Nucleo-F446RE board via USB type A to mini-B cable to the ST-LINK connector.
- Ensure jumpers are fitted on the ST-LINK connector CN2.
- Configure software parameters via configuration files like `sys_conf.h`, `radio_conf.h`, `lorawan_conf.h`, `lora_app.h`, `se-identity.h`, `mw_log_conf.h`, `radio_board_if.h`, and `main.h`.
- Make sure the region and class settings in `lora_app.h` are compatible with those in `lorawan_conf.h`.

#### Setup Diagram
  -Set Up:
             --------------------------  V    V  --------------------------
             |      LoRa Object       |  |    |  |      LoRa Network      |
             |                        |  |    |  |                        |
   ComPort<--|                        |--|    |--|                        |-->Web Server
             |                        |          |                        |
             --------------------------          --------------------------


## How to Use

1. Open your preferred toolchain.
2. Rebuild all files and load the image into the target memory.
3. Run the example.
4. Open a terminal connected to the LoRa Object with UART configured as:
   - Baud rate: 115200
   - Data bits: 8
   - Stop bit: 1
   - Parity: None
   - Flow control: None

---

*Note: The application is designed specifically for the STMicroelectronics NUCLEO-F446RE device.*




