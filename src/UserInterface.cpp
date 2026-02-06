#include "UserInterface.h"

UserInterface::UserInterface() : statusLEDState(false) {}

void UserInterface::begin() {
  setupPins();
}

void UserInterface::setupPins() {
  DDRD |= (1 << PIN_DBG_LED_R) | (1 << PIN_DBG_LED_G);
  PORTD &= ~(1 << PIN_DBG_LED_R);
  PORTD &= ~(1 << PIN_DBG_LED_G);
  
  DDRB |= (1 << PIN_STOPBTN_LED_R) | (1 << PIN_STOPBTN_LED_G);
  PORTB |= (1 << PIN_STOPBTN_LED_R);
  PORTB |= (1 << PIN_STOPBTN_LED_G);
  
  DDRD |= (1 << PIN_BUZZER);
  PORTD &= ~(1 << PIN_BUZZER);
}

void UserInterface::updateStatusLED() {
  statusLEDState = !statusLEDState;
  bitWrite(PORTD, PIN_DBG_LED_G, statusLEDState);
}

void UserInterface::setStopBtnLED(StopBtnLEDState state) {
  switch (state) {
    case OFF:
      PORTB |= (1 << PIN_STOPBTN_LED_R);
      PORTB |= (1 << PIN_STOPBTN_LED_G);
      break;
    case GREEN:
      PORTB |= (1 << PIN_STOPBTN_LED_R);
      PORTB &= ~(1 << PIN_STOPBTN_LED_G);
      break;
    case YELLOW:
      PORTB &= ~(1 << PIN_STOPBTN_LED_R);
      PORTB &= ~(1 << PIN_STOPBTN_LED_G);
      break;
    case RED:
      PORTB &= ~(1 << PIN_STOPBTN_LED_R);
      PORTB |= (1 << PIN_STOPBTN_LED_G);
      break;
  }
}

void UserInterface::setStatusLED(bool on) {
  // Green LED
  if (on)
    PORTD |= (1 << PIN_DBG_LED_G);
  else
    PORTD &= ~(1 << PIN_DBG_LED_G);
}

void UserInterface::setDebugLED(bool on) {
  // Red LED
  if (on)
    PORTD |= (1 << PIN_DBG_LED_R);
  else
    PORTD &= ~(1 << PIN_DBG_LED_R);
}

void UserInterface::playBeep(uint16_t frequencyHz, uint16_t durationMs) {
  uint32_t halfPeriodUs = 500000UL / frequencyHz;
  uint32_t cycles = ((uint32_t)frequencyHz * durationMs) / 1000;
  
  for (uint32_t i = 0; i < cycles; i++) {
    PORTD |= (1 << PIN_BUZZER);
    delayMicroseconds(halfPeriodUs);
    PORTD &= ~(1 << PIN_BUZZER);
    delayMicroseconds(halfPeriodUs);
  }
}