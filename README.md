# robotont-firmware-power-management

This repository contains the firmware for the power management microcontroller (ATTiny88) of the Robotont robot. The firmware is written to be used with the Arduino IDE that can be downloaded from [here](https://www.arduino.cc/en/Main/Software).


## Setting up Arduino as ISP (In-System Programmer)

In order to upload the firmware to the power management microcontroller, a programming device is required. As a cost-effective and easy-to-use solution, one could use an Arduino Nano board and upload the ArduinoISP example sketch to it.
The Arduino Nano board has to be then connected to the power management microcontroller using a 2x3 pin header located under the OLED display of the Robotont mainboard. Follow the signal mapping below to connect the Arduino Nano board to the standard 2x3 ISP header:

| Arduino board | Robotont PWR MGMT PROG header |
|---------------|-------------------------------|
| 3.3V          | VCC                           |
| GND           | GND                           |
| 13            | SCK                           |
| 12            | MISO                          |
| 11            | MOSI                          |
| 10            | RESET                         |

Open the ArduinoISP example sketch from the Arduino IDE and upload it to the Arduino Nano board. After the sketch has been uploaded, go to Tools -> Programmer and select "Arduino as ISP". The Arduino Nano board is now ready to be used as a programmer.

## Uploading the firmware

For uploading the firmware to the power management microcontroller, the ATTinyCore library must be installed and the board configured.

### Installing the ATTinyCore library

In Arduino IDE, go to File -> Preferences -> Additional boards manager URLs and add the following URL:

  ```
  https://raw.githubusercontent.com/damellis/attiny/ide-1.6.x-boards-manager/package_damellis_attiny_index.json
  ```
    
Then go to *Tools* -> *Board* -> *Boards Manager* and search for "ATTinyCore" and install it.

### Selecting the board and the programmer settings

Under Tools menu, select the following settings:

- *Board* -> *ATTinyCore* -> *ATtiny48/88(No bootloader)*

- *Chip* -> *ATtiny88*

- *Clock Source* -> *1 MHz (internal)*

- *Pin mapping* -> *Standard*

- *LTO* -> *Enabled*

- *Programmer* -> *Arduino as ISP*

### Uploading
For uploading the firmware to the power management microcontroller, open the firmware sketch localed in this repository with the Arduino IDE and go to:
- *Sketch* -> *Upload Using Programmer*

Once the firmware has been uploaded, the ATTiny88 microcontroller resets and the firmware starts running.


## Firmware functionality
The firmware is responsible for monitoring the battery status, managing power switching, and communicating with the main controller via I2C protocol. UI elements such as power button, buzzer, and status LEDs are used to provide user feedback. The firmware reads voltage and current sensors, as well as the battery pack info and sends this information to the main controller for further processing.

### I2C Data Packet Structure

**Total size: 24 bytes (all values big-endian uint16_t)**

| Bytes | Field | Description |
|-------|-------|-------------|
| 0-1 | Motor Current | Motor current reading |
| 2-3 | NUC Current | NUC current reading |
| 4-5 | Voltage | System voltage |
| 6-7 | Battery Voltage | Battery voltage |
| 8-9 | Pack Voltage | Battery pack total voltage |
| 10-19 | Cell Voltages[5] | Individual cell voltages (5 cells × 2 bytes) |
| 20-21 | Cell Temperature | Temperature measured from the cells (Degrees Celcius) |
| 22-23 | Mosfet Temperature | Temperature measured from the mosfet (Degrees Celcius) |


#### Notes

- All values are 16-bit unsigned integers in big-endian format (high byte first).
- The data packet is sent periodically (every 100 ms) to the main controller.