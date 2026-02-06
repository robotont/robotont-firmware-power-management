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
  void initTWI();
  void sendData(const SensorData& sensorData, const BatteryData& batteryData);
  
private:
  static const uint8_t MAX_RETRY_ATTEMPTS = 3;
  bool attemptTransmission(const byte* data, size_t length);
  void resetBus();
  bool busStuck();
  
  // Open-drain helpers
  inline void sdaLow();
  inline void sdaRelease();
  inline void sclLow();
  inline void sclRelease();
  inline bool sdaRead();
  inline bool sclRead();
  bool waitSclHigh(uint16_t timeoutUs);
};

#endif