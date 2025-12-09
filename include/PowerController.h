#ifndef POWER_CONTROLLER_H
#define POWER_CONTROLLER_H

#include <Arduino.h>
#include "Config.h"
#include "SensorManager.h"

class PowerController {
public:
  PowerController();
  void begin(SensorManager& sensors);
  void update(unsigned long currentTime, SensorManager& sensors);
  bool isSysPowerOn() const { return sysPowerOn; }
  
private:
  bool sysPowerOn;
  bool motorPowerOn;
  uint8_t prevStatus;
  unsigned long buttonPressStartTime;
  bool actionTaken;
  
  void setupPins();
  void handlePowerButton(unsigned long currentTime, uint8_t status, SensorManager& sensors);
  void handleStatusChanges(uint8_t status, uint8_t changed, SensorManager& sensors);
  
  void setSysPower(bool on, SensorManager& sensors);
  void setMotorPower(bool on, SensorManager& sensors);
  void updateStopBtnLED(uint8_t status);
  
  void playPowerOnSound();
  void playPowerOffSound();
  void playWallPowerConnectSound(uint8_t status);
  void playWallPowerDisconnectSound(uint8_t status);
  void playStopBtnSound(uint8_t status);
};

#endif