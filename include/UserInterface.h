#ifndef USER_INTERFACE_H
#define USER_INTERFACE_H

#include <Arduino.h>
#include "Config.h"
#include "PowerController.h"

class UserInterface {
public:
  UserInterface();
  void begin();
  void updateStatusLED();
  void updateEStopLED(bool eStopPressed);
  void playBeep(uint16_t frequency);
  
private:
  bool statusLEDState;
  void setupPins();
};

#endif