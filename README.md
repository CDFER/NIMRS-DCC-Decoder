# NIMRS-DCC-Decoder

## Overview

The NIMRS-DCC-Decoder is a project designed to decode and control DCC (Digital Command Control) signals for model railroads using an ESP32 microcontroller. It leverages the NmraDcc library for DCC signal decoding and provides motor control, LED indication, and OTA (Over-The-Air) updates.

## Features

- **DCC Signal Decoding**: Supports short and long DCC addresses.
- **Motor Control**: PWM-based motor control with configurable speed steps.
- **LED Indication**: Onboard LED control via DCC function packets.
- **NeoPixel Support**: Control NeoPixel LEDs for visual feedback.
- **WiFi Connectivity**: Connects to a WiFi network for OTA updates and Telnet communication.
- **OTA Updates**: Update firmware wirelessly.
- **Telnet Debugging**: Monitor and debug via Telnet.

## Hardware Requirements

- ESP32 microcontroller.
- Motor driver (e.g., H-bridge).
- NeoPixel LED strip (optional).
- DCC signal input.
- Power supply.

## Software Requirements

- [PlatformIO](https://platformio.org/) IDE.
- Arduino framework.

## Setup Instructions

1. Clone this repository:

   ```bash
   git clone https://github.com/yourusername/NIMRS-DCC-Decoder.git
   ```

2. Open the project in PlatformIO.
3. Rename `include/secrets-blank.h` to `secrets.h` and update it with your WiFi credentials:

   ```cpp
   const char* ssid = "YourWiFiSSID";
   const char* password = "YourWiFiPassword";
   ```

4. Connect your ESP32 to your computer.
5. Build and upload the firmware using PlatformIO.

## Configuration

The project can be configured via the `platformio.ini` file. Key configurations include:

- **Build Flags**: Define debug levels, OTA settings, and decoder version.
- **Environment Settings**: Configure upload protocols, monitor ports, and speeds.

## Usage

- **Motor Control**: The motor is controlled via DCC speed packets.
- **LED Indication**: The onboard LED can be toggled using DCC function packets.
- **NeoPixel LEDs**: Visual feedback can be customized using NeoPixel LEDs.
- **Telnet Debugging**: Connect to the ESP32 via Telnet to monitor and debug.

## Libraries Used

- [NmraDcc](https://github.com/mrrwa/NmraDcc)
- [NeoPixelBus](https://github.com/Makuna/NeoPixelBus)
- [ESP Telnet](https://github.com/lennarthennigs/ESP-Telnet)

## License

This project is licensed under the MIT License. See the LICENSE file for details.

## Acknowledgments

Special thanks to the authors of the libraries used in this project for their excellent work.
