#include "SensorManager.h"
#include <avr/interrupt.h>

SensorManager* g_sensorManager = nullptr;

SensorManager::SensorManager()
  : voltage(0), batteryVoltage(0), nucCurrent(0), motorCurrent(0),
    eStopPressed(false), currentChannel(ADC_VOLTAGE), adcCounter(0) {
  g_sensorManager = this;
}

void SensorManager::begin() {
  setupPins();
  setupADC();
  eStopPressed = !(PIND & (1 << PIN_ESTOP_SW));
  PCMSK2 |= (1 << PCINT19);
  PCICR |= (1 << PCIE2);
}

void SensorManager::setupPins() {
  DDRC &= ~(1 << PIN_I_NUC_SENSE);
  DDRA &= ~(1 << PIN_I_MOTORS_SENSE);
  DDRC &= ~(1 << PIN_V_SENSE);
  DDRC &= ~(1 << PIN_VBAT_SENSE);
  DDRD &= ~(1 << PIN_POWER_SW);
  DDRD &= ~(1 << PIN_ESTOP_SW);
  DDRB &= ~(1 << PIN_PWR_SRC_SENSE);
}

void SensorManager::setupADC() {
  ADCSRA = (1 << ADEN) | (1 << ADIE) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
  ADCSRB = 0;
  ADMUX = (1 << REFS0) | (PIN_V_SENSE_ADC & 0x0F);
  ADCSRA |= (1 << ADSC);
}

void SensorManager::selectADCChannel(uint8_t channel) {
  ADMUX = (ADMUX & 0xF0) | (channel & 0x0F);
}

SensorData SensorManager::getAllReadings() const {
  SensorData data;
  cli();
  data.voltage = voltage;
  data.batteryVoltage = batteryVoltage;
  data.nucCurrent = nucCurrent;
  data.motorCurrent = motorCurrent;
  sei();
  return data;
}

ISR(ADC_vect) {
  if (g_sensorManager == nullptr) return;
  
  uint8_t low = ADCL;
  uint8_t high = ADCH;
  uint16_t adcValue = (high << 8) | low;
  
  switch (g_sensorManager->currentChannel) {
    case ADC_VOLTAGE:
      g_sensorManager->voltage = adcValue;
      if (g_sensorManager->adcCounter < 200) {
        g_sensorManager->selectADCChannel(PIN_VBAT_SENSE_ADC);
        g_sensorManager->currentChannel = ADC_BATTERY_VOLTAGE;
      } else {
        g_sensorManager->selectADCChannel(PIN_I_NUC_SENSE_ADC);
        g_sensorManager->currentChannel = ADC_NUC_CURRENT;
      }
      break;
      
    case ADC_BATTERY_VOLTAGE:
      g_sensorManager->batteryVoltage = adcValue;
      g_sensorManager->selectADCChannel(PIN_V_SENSE_ADC);
      g_sensorManager->currentChannel = ADC_VOLTAGE;
      break;
      
    case ADC_NUC_CURRENT:
      g_sensorManager->nucCurrent = adcValue;
      g_sensorManager->selectADCChannel(PIN_I_MOTORS_SENSE_ADC);
      g_sensorManager->currentChannel = ADC_MOTOR_CURRENT;
      break;
      
    case ADC_MOTOR_CURRENT:
      g_sensorManager->motorCurrent = adcValue;
      g_sensorManager->selectADCChannel(PIN_VBAT_SENSE_ADC);
      g_sensorManager->currentChannel = ADC_BATTERY_VOLTAGE;
      g_sensorManager->adcCounter = 0;
      break;
  }
  
  g_sensorManager->adcCounter++;
  ADCSRA |= (1 << ADSC);
}

ISR(PCINT2_vect) {
  if (g_sensorManager != nullptr) {
    g_sensorManager->eStopPressed = !(PIND & (1 << PIN_ESTOP_SW));
  }
}