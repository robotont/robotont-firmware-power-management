#ifndef I2C_COMMUNICATION_H
#define I2C_COMMUNICATION_H

#include <Arduino.h>
#include <Wire.h>
#include "Config.h"
#include "SensorManager.h"
#include "MakitaBattery.h"

class I2CCommunication {
public:
  I2CCommunication();
  void begin();
  void sendData(const SensorData& sensorData, const BatteryDebugData& batteryData);
  
private:
  static const uint8_t MAX_RETRY_ATTEMPTS = 3;
  bool attemptTransmission(const byte* data, size_t length);
  void recoverBus();
};

#endif