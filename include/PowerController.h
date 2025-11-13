#ifndef POWER_CONTROLLER_H
#define POWER_CONTROLLER_H

#include <Arduino.h>
#include "Config.h"

enum PowerState { POWER_OFF, POWER_ON };
enum SwitchingState { CONNECTED_TO_WALL, CONNECTED_TO_BATTERY };

class PowerController {
public:
  PowerController();
  void begin();
  void update(unsigned long currentTime, bool eStopPressed);
  PowerState getState() const { return state; }
  SwitchingState getSwitchingState() const { return switchingState; }
  
private:
  PowerState state;
  SwitchingState switchingState;
  unsigned long buttonPressStartTime;
  bool eStopPreviousState;
  bool actionTaken;
  
  void setupPins();
  void switchToBattery(bool eStopPressed);
  void switchToWall();
  void powerOn(bool eStopPressed);
  void powerOff();
  void handlePowerButton(unsigned long currentTime);
  bool isPowerButtonPressed();
  void handlePowerSourceMonitoring(bool eStopPressed);
};

#endif