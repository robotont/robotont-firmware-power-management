#ifndef POWER_CONTROLLER_H
#define POWER_CONTROLLER_H

#include <Arduino.h>
#include "Config.h"

enum PowerState { POWER_OFF, POWER_ON };

class PowerController {
public:
  PowerController();
  void begin();
  void update(unsigned long currentTime, bool stopBtnPressed, bool wallPowerPresent);
  PowerState getState() const { return state; }
  
private:
  PowerState state;
  unsigned long buttonPressStartTime;
  bool stopBtnPreviousState;
  bool wallPowerPreviousState;
  bool actionTaken;
  
  void setupPins();
  void sysPowerOn(bool stopBtnPressed, bool wallPowerPresent);
  void sysPowerOff();
  void motorPowerOn();
  void motorPowerOff();
  void handlePowerButton(unsigned long currentTime);
  void updateStopBtnLED(bool stopBtnPressed, bool wallPowerPresent);
  bool isPowerButtonPressed();
};

#endif