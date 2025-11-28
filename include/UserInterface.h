#ifndef USER_INTERFACE_H
#define USER_INTERFACE_H

#include <Arduino.h>
#include "Config.h"
#include "PowerController.h"

class UserInterface {
public:
  enum StopBtnLEDState { OFF, GREEN, YELLOW, RED };
  
  UserInterface();
  void begin();
  void updateStatusLED();
  void setStopBtnLED(StopBtnLEDState state);
  void setStatusLED(bool on);
  void setDebugLED(bool on);
  void playBeep(uint16_t frequencyHz, uint16_t durationMs = 100);
  
private:
  bool statusLEDState;
  void setupPins();
};

#endif