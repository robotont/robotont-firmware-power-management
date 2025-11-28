#ifndef MAKITA_BATTERY_H
#define MAKITA_BATTERY_H

#include <Arduino.h>
#include <OneWire2.h>
#include "Config.h"

struct BatteryData {
  uint16_t packVoltage;
  uint16_t cellVoltages[5];
  uint16_t temperatures[2];
  BatteryData() { memset(this, 0, sizeof(*this)); }
};

class MakitaBattery {
public:
  MakitaBattery(uint8_t enablePin, uint8_t dataPin);
  void begin();
  void update();
  BatteryData getData() const { return data; }
  
private:
  uint8_t enablePin;
  uint8_t dataPin;
  OneWire oneWire;
  BatteryData data;
  uint8_t debugCounter;
  
  void enableBattery();
  void disableBattery();
  void sendCommand(const byte* cmd, uint8_t cmdLen, byte* response, uint8_t responseLen);
  void sendClearCommand();
};

#endif