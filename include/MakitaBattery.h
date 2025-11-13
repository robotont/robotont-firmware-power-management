#ifndef MAKITA_BATTERY_H
#define MAKITA_BATTERY_H

#include <Arduino.h>
#include <OneWire2.h>
#include "Config.h"

#define BATTERY_CLEAR_CMD_0 0xF0
#define BATTERY_CLEAR_CMD_1 0x00
#define BATTERY_READ_CELL_1_CMD 0x31
#define BATTERY_SKIP_ROM_CMD 0xCC
#define BATTERY_FUNCTION_CMD 0x99

struct BatteryDebugData {
  uint8_t data[8];
  BatteryDebugData() { memset(data, 0, sizeof(data)); }
};

class MakitaBattery {
public:
  MakitaBattery(uint8_t enablePin, uint8_t dataPin);
  void begin();
  void update();
  BatteryDebugData getDebugData() const { return debugData; }
  
private:
  uint8_t enablePin;
  uint8_t dataPin;
  OneWire oneWire;
  BatteryDebugData debugData;
  uint8_t debugCounter;
  
  void enableBattery();
  void disableBattery();
  void sendCommand(const byte* cmd, uint8_t cmdLen, byte* response, uint8_t responseLen);
  void disableInterrupts();
  void enableInterrupts();
};

#endif