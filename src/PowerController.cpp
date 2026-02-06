#include "PowerController.h"
#include "UserInterface.h"

extern UserInterface ui;

PowerController::PowerController()
  : sysPowerOn(false), motorPowerOn(false), prevStatus(0),
    buttonPressStartTime(0), actionTaken(false) {}

void PowerController::begin(SensorManager& sensors) {
  cli();  // Disable interrupts during init
  
  setupPins();
  
  // System starts OFF - ensure hardware matches
  sysPowerOn = false;
  motorPowerOn = false;
  PORTA &= ~(1 << PIN_SYS_PWR_CTRL);
  PORTC &= ~(1 << PIN_MOTOR_PWR_CTRL);
  
  // Sync sensor status flags with actual hardware state
  sensors.setSysPower(false);
  sensors.setMotorPower(false);
  
  // Read current input states
  prevStatus = sensors.getStatus();
  
  // Initialize button timing
  buttonPressStartTime = millis();
  actionTaken = false;
  
  // Set LED to OFF (system not powered)
  ui.setStopBtnLED(UserInterface::OFF);
  
  sei();  // Re-enable interrupts
}

void PowerController::setupPins() {
  DDRC |= (1 << PIN_MOTOR_PWR_CTRL);
  DDRA |= (1 << PIN_SYS_PWR_CTRL);
  PORTC &= ~(1 << PIN_MOTOR_PWR_CTRL);
  PORTA &= ~(1 << PIN_SYS_PWR_CTRL);
}

void PowerController::update(unsigned long currentTime, SensorManager& sensors) {
  uint8_t status = sensors.getStatus();
  uint8_t changed = status ^ prevStatus;
  
  handlePowerButton(currentTime, status, sensors);
  
  if (sysPowerOn && changed) {
    handleStatusChanges(status, changed, sensors);
  }
  
  prevStatus = status;
}

void PowerController::handlePowerButton(unsigned long currentTime, uint8_t status, SensorManager& sensors) {
  // Read power button directly from pin (not from status byte which isn't updated by ISR)
  bool buttonPressed = (PIND & (1 << PIN_POWER_SW)) != 0;
  
  if (buttonPressed) {
    buttonPressStartTime = currentTime;
    actionTaken = false;
    return;
  }
  
  if (actionTaken) {
    return;
  }
  
  unsigned long releasedDuration = currentTime - buttonPressStartTime;
  
  if (!sysPowerOn && releasedDuration >= POWER_ON_HOLD_TIME) {
    setSysPower(true, sensors);
    
    // Enable motors if conditions allow
    bool stopBtn = status & Status::STOP_BTN_MASK;
    bool wallPower = status & Status::WALL_POWER_MASK;
    if (!wallPower && !stopBtn) {
      setMotorPower(true, sensors);
    }
    
    updateStopBtnLED(status);
    playPowerOnSound();
    actionTaken = true;
    
  } else if (sysPowerOn && releasedDuration >= POWER_OFF_HOLD_TIME) {
    setMotorPower(false, sensors);
    setSysPower(false, sensors);
    ui.setStopBtnLED(UserInterface::OFF);
    playPowerOffSound();
    actionTaken = true;
  }
}

void PowerController::handleStatusChanges(uint8_t status, uint8_t changed, SensorManager& sensors) {
  bool stopBtn = status & Status::STOP_BTN_MASK;
  bool wallPower = status & Status::WALL_POWER_MASK;
  
  if (changed & Status::WALL_POWER_MASK) {
    if (wallPower) {
      // Switched to wall power: motors OFF
      setMotorPower(false, sensors);
      playWallPowerConnectSound(status);
    } else {
      // Switched to battery
      setMotorPower(false, sensors);  // Turn off first
      if (!stopBtn) {
        setMotorPower(true, sensors);  // Enable if stop button not pressed
      }
      playWallPowerDisconnectSound(status);
    }
    updateStopBtnLED(status);
    
  } else if (changed & Status::STOP_BTN_MASK) {
    // Stop button changed without power source change
    if (stopBtn) {
      setMotorPower(false, sensors);
    } else if (!wallPower) {
      setMotorPower(true, sensors);
    }
    updateStopBtnLED(status);
    playStopBtnSound(status);
  }
}

void PowerController::setSysPower(bool on, SensorManager& sensors) {
  sysPowerOn = on;
  if (on) {
    PORTA |= (1 << PIN_SYS_PWR_CTRL);
  } else {
    PORTA &= ~(1 << PIN_SYS_PWR_CTRL);
  }
  sensors.setSysPower(on);
  ui.setDebugLED(on);
}

void PowerController::setMotorPower(bool on, SensorManager& sensors) {
  motorPowerOn = on;
  if (on) {
    PORTC |= (1 << PIN_MOTOR_PWR_CTRL);
  } else {
    PORTC &= ~(1 << PIN_MOTOR_PWR_CTRL);
  }
  sensors.setMotorPower(on);
}

void PowerController::updateStopBtnLED(uint8_t status) {
  bool stopBtn = status & Status::STOP_BTN_MASK;
  bool wallPower = status & Status::WALL_POWER_MASK;
  
  if (stopBtn) {
    ui.setStopBtnLED(UserInterface::RED);
  } else if (wallPower) {
    ui.setStopBtnLED(UserInterface::YELLOW);
  } else {
    ui.setStopBtnLED(UserInterface::GREEN);
  }
}

void PowerController::playPowerOnSound() {
  for (uint16_t i = 500; i <= 1500; i += 50) {
    ui.playBeep(i, 20);
  }
}

void PowerController::playPowerOffSound() {
  for (uint16_t i = 1500; i >= 500; i -= 50) {
    ui.playBeep(i, 20);
  }
}

void PowerController::playWallPowerConnectSound(uint8_t status) {
  bool stopBtn = status & Status::STOP_BTN_MASK;
  
  ui.playBeep(BEEP_FREQ_HIGH, 150);
  ui.playBeep(BEEP_FREQ_LOW, 150);
  delay(100);
  
  if (stopBtn) {
    ui.playBeep(BEEP_FREQ_LOW, 100);   // RED LED - motors disabled
  } else {
    ui.playBeep(BEEP_FREQ_MED, 100);   // YELLOW LED - motors disabled (wall power)
  }
}

void PowerController::playWallPowerDisconnectSound(uint8_t status) {
  bool stopBtn = status & Status::STOP_BTN_MASK;
  
  ui.playBeep(BEEP_FREQ_LOW, 150);
  ui.playBeep(BEEP_FREQ_HIGH, 150);
  delay(100);
  
  if (stopBtn) {
    ui.playBeep(BEEP_FREQ_LOW, 100);   // RED LED - motors disabled
  } else {
    ui.playBeep(BEEP_FREQ_HIGH, 100);  // GREEN LED - motors enabled
  }
}

void PowerController::playStopBtnSound(uint8_t status) {
  bool stopBtn = status & Status::STOP_BTN_MASK;
  bool wallPower = status & Status::WALL_POWER_MASK;
  
  if (stopBtn) {
    ui.playBeep(BEEP_FREQ_LOW, 150);   // RED LED - motors disabled
  } else if (wallPower) {
    ui.playBeep(BEEP_FREQ_MED, 150);   // YELLOW LED - motors disabled (wall power)
  } else {
    ui.playBeep(BEEP_FREQ_HIGH, 150);  // GREEN LED - motors enabled
  }
}