#include "UserInterface.h"

UserInterface::UserInterface() : statusLEDState(false) {}

void UserInterface::begin() {
  setupPins();
}

void UserInterface::setupPins() {
  DDRD |= (1 << PIN_DBG_LED_R) | (1 << PIN_DBG_LED_G);
  bitWrite(PORTD, PIN_DBG_LED_R, 0);
  bitWrite(PORTD, PIN_DBG_LED_G, 0);
  
  DDRB |= (1 << PIN_STOPBTN_LED_R) | (1 << PIN_STOPBTN_LED_G);
  bitWrite(PORTB, PIN_STOPBTN_LED_R, 1);
  bitWrite(PORTB, PIN_STOPBTN_LED_G, 1);
  
  DDRD |= (1 << PIN_BUZZER);
  bitWrite(PORTD, PIN_BUZZER, 0);
}

void UserInterface::updateStatusLED() {
  statusLEDState = !statusLEDState;
  bitWrite(PORTD, PIN_DBG_LED_G, statusLEDState);
}

void UserInterface::setStopBtnLED(StopBtnLEDState state) {
  switch (state) {
    case OFF:
      bitWrite(PORTB, PIN_STOPBTN_LED_R, 1);
      bitWrite(PORTB, PIN_STOPBTN_LED_G, 1);
      break;
    case GREEN:
      bitWrite(PORTB, PIN_STOPBTN_LED_R, 1);
      bitWrite(PORTB, PIN_STOPBTN_LED_G, 0);
      break;
    case YELLOW:
      bitWrite(PORTB, PIN_STOPBTN_LED_R, 0);
      bitWrite(PORTB, PIN_STOPBTN_LED_G, 0);
      break;
    case RED:
      bitWrite(PORTB, PIN_STOPBTN_LED_R, 0);
      bitWrite(PORTB, PIN_STOPBTN_LED_G, 1);
      break;
  }
}

void UserInterface::setStatusLED(bool on) {
  // Green LED
  bitWrite(PORTD, PIN_DBG_LED_G, on);
}

void UserInterface::setDebugLED(bool on) {
  // Red LED
  bitWrite(PORTD, PIN_DBG_LED_R, on);
}

void UserInterface::playBeep(uint16_t frequencyHz, uint16_t durationMs) {
  uint32_t halfPeriodUs = 500000UL / frequencyHz;
  uint32_t cycles = ((uint32_t)frequencyHz * durationMs) / 1000;
  
  for (uint32_t i = 0; i < cycles; i++) {
    bitWrite(PORTD, PIN_BUZZER, HIGH);
    delayMicroseconds(halfPeriodUs);
    bitWrite(PORTD, PIN_BUZZER, LOW);
    delayMicroseconds(halfPeriodUs);
  }
}