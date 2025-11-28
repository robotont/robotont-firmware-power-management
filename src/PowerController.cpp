#include "PowerController.h"
#include "UserInterface.h"

extern UserInterface ui;

PowerController::PowerController()
  : state(POWER_OFF), buttonPressStartTime(0),
    stopBtnPreviousState(false), wallPowerPreviousState(false), actionTaken(false) {}

void PowerController::begin() {
  setupPins();
  buttonPressStartTime = millis();
  
  // Read initial states
  bool stopBtn = !(PIND & (1 << PIN_STOPBTN_SW));
  bool wallPower = (PINB & (1 << PIN_PWR_SRC_SENSE));
  
  stopBtnPreviousState = stopBtn;
  wallPowerPreviousState = wallPower;
  
  // Set initial LED state (system is OFF)
  ui.setStopBtnLED(UserInterface::OFF);
  
  // Call update once to sync state
  update(millis(), stopBtn, wallPower);
}

void PowerController::setupPins() {
  DDRC |= (1 << PIN_MOTOR_PWR_CTRL);
  DDRA |= (1 << PIN_SYS_PWR_CTRL);
  bitWrite(PORTC, PIN_MOTOR_PWR_CTRL, 0);
  bitWrite(PORTA, PIN_SYS_PWR_CTRL, 0);
}

void PowerController::update(unsigned long currentTime, bool stopBtnPressed, bool wallPowerPresent) {
  bool stopBtnChanged = (stopBtnPressed != stopBtnPreviousState);
  stopBtnPreviousState = stopBtnPressed;
  
  bool wallPowerChanged = (wallPowerPresent != wallPowerPreviousState);
  wallPowerPreviousState = wallPowerPresent;
  
  handlePowerButton(currentTime);
  
  if (state == POWER_ON) {
    if (wallPowerChanged) {
      if (wallPowerPresent) {
        // Switched to wall power: motors OFF
        motorPowerOff();
        ui.playBeep(BEEP_FREQ_HIGH, 150);
        ui.playBeep(BEEP_FREQ_LOW, 150);
        delay(100);
        // Final beep matches LED state
        if (stopBtnPressed) {
          ui.playBeep(BEEP_FREQ_LOW, 100);  // RED LED - motors disabled
        } else {
          ui.playBeep(BEEP_FREQ_MED, 100);  // YELLOW LED - motors disabled
        }
      } else {
        // Switched to battery
        motorPowerOff();  // Turn off first, then check if should enable
        ui.playBeep(BEEP_FREQ_LOW, 150);
        ui.playBeep(BEEP_FREQ_HIGH, 150);
        delay(100);
        if (stopBtnPressed) {
          ui.playBeep(BEEP_FREQ_LOW, 100);  // RED LED - motors disabled
        } else {
          motorPowerOn();  // Enable motors
          ui.playBeep(BEEP_FREQ_HIGH, 100);  // GREEN LED - motors enabled
        }
      }
      
      updateStopBtnLED(stopBtnPressed, wallPowerPresent);
    }
    else if (stopBtnChanged) {
      // Stop button changed without power source change
      if (stopBtnPressed) {
        motorPowerOff();
        ui.playBeep(BEEP_FREQ_LOW, 150);  // RED LED - motors disabled
      } else {
        if (!wallPowerPresent) {
          motorPowerOn();
          ui.playBeep(BEEP_FREQ_HIGH, 150);  // GREEN LED - motors enabled
        } else {
          ui.playBeep(BEEP_FREQ_MED, 150);  // YELLOW LED - motors disabled (wall power)
        }
      }
      updateStopBtnLED(stopBtnPressed, wallPowerPresent);
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
    sysPowerOn(stopBtnPreviousState, wallPowerPreviousState);
    actionTaken = true;
  } else if (state == POWER_ON && releasedDuration >= POWER_OFF_HOLD_TIME) {
    sysPowerOff();
    actionTaken = true;
  }
}

bool PowerController::isPowerButtonPressed() {
  return (PIND & (1 << PIN_POWER_SW)) != 0;
}



void PowerController::sysPowerOn(bool stopBtnPressed, bool wallPowerPresent) {
  state = POWER_ON;
  bitWrite(PORTA, PIN_SYS_PWR_CTRL, 1);
  
  ui.setDebugLED(true);
  updateStopBtnLED(stopBtnPressed, wallPowerPresent);
  
  if (!wallPowerPresent && !stopBtnPressed) {
    motorPowerOn();
  }
  
  // Ascending sweep: system starting up
  for (uint16_t i = 500; i <= 1500; i += 50) {
    ui.playBeep(i, 20);
  }
}

void PowerController::sysPowerOff() {
  state = POWER_OFF;
  motorPowerOff();
  bitWrite(PORTA, PIN_SYS_PWR_CTRL, 0);
  
  ui.setDebugLED(false);
  updateStopBtnLED(false, false);  // OFF state
  
  // Descending: system shutting down
  // Ascending sweep: system starting up
  for (uint16_t i = 1500; i >= 500; i -= 50) {
    ui.playBeep(i, 20);
  }
}

void PowerController::motorPowerOn() {
  bitWrite(PORTC, PIN_MOTOR_PWR_CTRL, 1);
}

void PowerController::motorPowerOff() {
  bitWrite(PORTC, PIN_MOTOR_PWR_CTRL, 0);
}

void PowerController::updateStopBtnLED(bool stopBtnPressed, bool wallPowerPresent) {
  if (stopBtnPressed) {
    ui.setStopBtnLED(UserInterface::RED);
  } else if (wallPowerPresent) {
    ui.setStopBtnLED(UserInterface::YELLOW);
  } else {
    ui.setStopBtnLED(UserInterface::GREEN);
  }
}