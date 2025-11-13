#include "PowerController.h"
#include "UserInterface.h"

extern UserInterface ui;

PowerController::PowerController()
  : state(POWER_OFF), switchingState(CONNECTED_TO_BATTERY), buttonPressStartTime(0),
    eStopPreviousState(false), actionTaken(false) {}

void PowerController::begin() {
  setupPins();
  buttonPressStartTime = millis();
}

void PowerController::setupPins() {
  DDRC |= (1 << PIN_MOTOR_PWR_CTRL);
  DDRA |= (1 << PIN_SYS_PWR_CTRL);
  bitWrite(PORTC, PIN_MOTOR_PWR_CTRL, 0);
  bitWrite(PORTA, PIN_SYS_PWR_CTRL, 0);
}

void PowerController::update(unsigned long currentTime, bool eStopPressed) {
  // Track E-stop state changes
  bool eStopChanged = (eStopPressed != eStopPreviousState);
  eStopPreviousState = eStopPressed;
  
  handlePowerButton(currentTime);
  
  if (state == POWER_ON) {
    handlePowerSourceMonitoring(eStopPressed);
    
    if (eStopChanged) {
      if (eStopPressed) {
        bitWrite(PORTC, PIN_MOTOR_PWR_CTRL, 0);
        ui.playBeep(BEEP_FREQ_MED);
        ui.updateEStopLED(true);
      } else if (switchingState == CONNECTED_TO_BATTERY) {
        bitWrite(PORTC, PIN_MOTOR_PWR_CTRL, 1);
        ui.playBeep(BEEP_FREQ_MED);
        ui.updateEStopLED(false);
      }
    }
  }
}

void PowerController::handlePowerButton(unsigned long currentTime) {
  bool buttonPressed = isPowerButtonPressed();
  
  if (buttonPressed) {
    buttonPressStartTime = currentTime;
    actionTaken = false;
    return;
  }
  
  if (actionTaken) {
    return;
  }
  
  unsigned long releasedDuration = currentTime - buttonPressStartTime;
  
  if (state == POWER_OFF && releasedDuration >= POWER_ON_HOLD_TIME) {
    powerOn(eStopPreviousState);
    actionTaken = true;
  } else if (state == POWER_ON && releasedDuration >= POWER_OFF_HOLD_TIME) {
    powerOff();
    actionTaken = true;
  }
}

bool PowerController::isPowerButtonPressed() {
  return (PIND & (1 << PIN_POWER_SW)) != 0;
}

void PowerController::powerOn(bool eStopPressed) {
  state = POWER_ON;
  bitWrite(PORTD, PIN_DBG_LED_R, 1);
  ui.playBeep(BEEP_FREQ_HIGH);
  ui.playBeep(BEEP_FREQ_MED);
  ui.playBeep(BEEP_FREQ_LOW);
  ui.updateEStopLED(eStopPressed);
  
  // Immediately switch to appropriate power source
  bool wallPowerPresent = (PINB & (1 << PIN_PWR_SRC_SENSE));
  if (wallPowerPresent) {
    switchToWall();
  } else {
    switchToBattery(eStopPressed);
  }
}

void PowerController::powerOff() {
  state = POWER_OFF;
  switchingState = CONNECTED_TO_BATTERY;
  bitWrite(PORTC, PIN_MOTOR_PWR_CTRL, 0);
  bitWrite(PORTA, PIN_SYS_PWR_CTRL, 0);
  bitWrite(PORTD, PIN_DBG_LED_R, 0);
  bitWrite(PORTB, PIN_ESTOP_LED_R, 1);  // Both OFF
  bitWrite(PORTB, PIN_ESTOP_LED_G, 1);
  ui.playBeep(BEEP_FREQ_LOW);
  ui.playBeep(BEEP_FREQ_MED);
  ui.playBeep(BEEP_FREQ_HIGH);
}

void PowerController::switchToBattery(bool eStopPressed) {
  bitWrite(PORTA, PIN_SYS_PWR_CTRL, 1);
  bitWrite(PORTC, PIN_MOTOR_PWR_CTRL, !eStopPressed);
  switchingState = CONNECTED_TO_BATTERY;
}

void PowerController::switchToWall() {
  bitWrite(PORTA, PIN_SYS_PWR_CTRL, 1);
  bitWrite(PORTC, PIN_MOTOR_PWR_CTRL, 0);
  switchingState = CONNECTED_TO_WALL;
}

void PowerController::handlePowerSourceMonitoring(bool eStopPressed) {
  bool wallPowerPresent = (PINB & (1 << PIN_PWR_SRC_SENSE));
  
  if (wallPowerPresent && switchingState != CONNECTED_TO_WALL) {
    switchToWall();
  } else if (!wallPowerPresent && switchingState != CONNECTED_TO_BATTERY) {
    switchToBattery(eStopPressed);
  }
}