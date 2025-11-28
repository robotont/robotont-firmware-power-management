#include <Arduino.h>
#include <avr/wdt.h>
#include <util/atomic.h>

#include "Config.h"
#include "MakitaBattery.h"
#include "PowerController.h"
#include "SensorManager.h"
#include "I2CCommunication.h"
#include "UserInterface.h"

MakitaBattery battery(PIN_BAT_EN, PIN_BAT_DATA);
PowerController powerController;
SensorManager sensors;
I2CCommunication i2cComm;
UserInterface ui;

unsigned long prevTimeForLed = 0;
unsigned long prevTimeForI2c = 0;
unsigned long prevTimeForBattery = 0;

const unsigned long LED_INTERVAL = 500;
const unsigned long I2C_INTERVAL = 200;
const unsigned long BATTERY_INTERVAL = 5000;

void setup() {
  sensors.begin();
  powerController.begin();
  ui.begin();
  battery.begin();
  i2cComm.begin();
  sei();
  
  prevTimeForLed = millis();
  prevTimeForI2c = millis();
  prevTimeForBattery = millis();
}

void loop() {
  wdt_reset();
  unsigned long now = millis();

  powerController.update(now, sensors.getStopBtnPressed(), sensors.getWallPowerPresent());
  
  if (now - prevTimeForLed > LED_INTERVAL) {
    prevTimeForLed = now;
    ui.updateStatusLED();
  }

  if (now - prevTimeForI2c > I2C_INTERVAL) {
    prevTimeForI2c = now;
    SensorData sensorData = sensors.getData();
    BatteryData batteryData = battery.getData();
    i2cComm.sendData(sensorData, batteryData);
  }

  if (now - prevTimeForBattery > BATTERY_INTERVAL) {
    prevTimeForBattery = now;
    battery.update();
  }
}