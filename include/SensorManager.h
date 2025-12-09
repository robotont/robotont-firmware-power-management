#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#include <Arduino.h>
#include "Config.h"

// Status byte bit positions
namespace Status {
  constexpr uint8_t STOP_BTN_BIT     = 0;  // bit 0: stop button pressed
  constexpr uint8_t POWER_SW_BIT     = 1;  // bit 1: power switch pressed
  constexpr uint8_t WALL_POWER_BIT   = 2;  // bit 2: wall power present
  constexpr uint8_t MOTOR_POWER_BIT  = 3;  // bit 3: motor power enabled
  constexpr uint8_t SYS_POWER_BIT    = 4;  // bit 4: system power on
  // bits 5-7: reserved for future use
  
  constexpr uint8_t STOP_BTN_MASK    = (1 << STOP_BTN_BIT);
  constexpr uint8_t POWER_SW_MASK    = (1 << POWER_SW_BIT);
  constexpr uint8_t WALL_POWER_MASK  = (1 << WALL_POWER_BIT);
  constexpr uint8_t MOTOR_POWER_MASK = (1 << MOTOR_POWER_BIT);
  constexpr uint8_t SYS_POWER_MASK   = (1 << SYS_POWER_BIT);
}

enum ADCChannelState { ADC_VOLTAGE, ADC_BATTERY_VOLTAGE, ADC_NUC_CURRENT, ADC_MOTOR_CURRENT };

struct SensorData {
  uint8_t status;
  uint16_t voltage;
  uint16_t batteryVoltage;
  uint16_t nucCurrent;
  uint16_t motorCurrent;
  
  SensorData() : status(0), voltage(0), batteryVoltage(0), nucCurrent(0), motorCurrent(0) {}
  
  // Status bit accessors
  bool isStopBtnPressed() const   { return status & Status::STOP_BTN_MASK; }
  bool isPowerSwPressed() const   { return status & Status::POWER_SW_MASK; }
  bool isWallPowerPresent() const { return status & Status::WALL_POWER_MASK; }
  bool isMotorPowerOn() const     { return status & Status::MOTOR_POWER_MASK; }
  bool isSysPowerOn() const       { return status & Status::SYS_POWER_MASK; }
};

class SensorManager {
public:
  SensorManager();
  void begin();
  
  // Get snapshot of all sensor data (interrupt-safe)
  SensorData getData() const;
  
  // Update output status bits (called by PowerController)
  void setMotorPower(bool on);
  void setSysPower(bool on);
  
  // Direct status access (for change detection)
  uint8_t getStatus() const;
  
  // ISR-accessible members (must be public for ISR access)
  volatile uint8_t status;
  volatile uint16_t voltage;
  volatile uint16_t batteryVoltage;
  volatile uint16_t nucCurrent;
  volatile uint16_t motorCurrent;
  volatile ADCChannelState currentChannel;
  
  void selectADCChannel(uint8_t channel);
  
private:
  void setupPins();
  void setupADC();
  void updateInputStatus();
};

extern SensorManager* g_sensorManager;

#endif