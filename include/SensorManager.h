#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#include <Arduino.h>
#include "Config.h"

enum ADCChannelState { ADC_VOLTAGE, ADC_BATTERY_VOLTAGE, ADC_NUC_CURRENT, ADC_MOTOR_CURRENT };

struct SensorData {
  uint16_t voltage;
  uint16_t batteryVoltage;
  uint16_t nucCurrent;
  uint16_t motorCurrent;
  SensorData() : voltage(0), batteryVoltage(0), nucCurrent(0), motorCurrent(0) {}
};

class SensorManager {
public:
  SensorManager();
  void begin();
  uint16_t getVoltage() const { return voltage; }
  uint16_t getBatteryVoltage() const { return batteryVoltage; }
  uint16_t getNucCurrent() const { return nucCurrent; }
  uint16_t getMotorCurrent() const { return motorCurrent; }
  bool getStopBtnPressed() const { return stopBtnPressed; }
  bool getWallPowerPresent() const { return wallPowerPresent; }
  SensorData getData() const;

  volatile uint16_t voltage;
  volatile uint16_t batteryVoltage;
  volatile uint16_t nucCurrent;
  volatile uint16_t motorCurrent;
  volatile bool stopBtnPressed;
  volatile bool wallPowerPresent;
  volatile ADCChannelState currentChannel;
  
  void selectADCChannel(uint8_t channel);

  
private:
  void setupADC();
  void setupPins();
};

extern SensorManager* g_sensorManager;

#endif