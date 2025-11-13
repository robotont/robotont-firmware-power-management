#include "UserInterface.h"

UserInterface::UserInterface() : statusLEDState(false) {}

void UserInterface::begin() {
  setupPins();
}

void UserInterface::setupPins() {
  DDRD |= (1 << PIN_DBG_LED_R) | (1 << PIN_DBG_LED_G);
  bitWrite(PORTD, PIN_DBG_LED_R, 0);
  bitWrite(PORTD, PIN_DBG_LED_G, 0);
  
  DDRB |= (1 << PIN_ESTOP_LED_R) | (1 << PIN_ESTOP_LED_G);
  bitWrite(PORTB, PIN_ESTOP_LED_R, 1);
  bitWrite(PORTB, PIN_ESTOP_LED_G, 1);
  
  DDRD |= (1 << PIN_BUZZER);
  bitWrite(PORTD, PIN_BUZZER, 0);
}

void UserInterface::updateStatusLED() {
  statusLEDState = !statusLEDState;
  bitWrite(PORTD, PIN_DBG_LED_G, statusLEDState);
}

void UserInterface::updateEStopLED(bool eStopPressed) {
  if (eStopPressed) {
    bitWrite(PORTB, PIN_ESTOP_LED_R, 0);
    bitWrite(PORTB, PIN_ESTOP_LED_G, 1);
  } else {
    bitWrite(PORTB, PIN_ESTOP_LED_R, 1);
    bitWrite(PORTB, PIN_ESTOP_LED_G, 0);
  }
}

void UserInterface::playBeep(uint16_t frequency) {
  for (uint8_t i = 0; i < 100; i++) {
    bitWrite(PORTD, PIN_BUZZER, HIGH);
    delayMicroseconds(frequency);
    bitWrite(PORTD, PIN_BUZZER, LOW);
    delayMicroseconds(frequency);
  }
}