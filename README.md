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
