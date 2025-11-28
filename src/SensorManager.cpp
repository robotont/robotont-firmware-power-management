#include "SensorManager.h"
#include <avr/interrupt.h>
#include <util/delay.h>

SensorManager* g_sensorManager = nullptr;

SensorManager::SensorManager()
  : voltage(0), batteryVoltage(0), nucCurrent(0), motorCurrent(0),
    stopBtnPressed(false), wallPowerPresent(false), currentChannel(ADC_VOLTAGE) {
  g_sensorManager = this;
}

void SensorManager::begin() {
  setupPins();
  setupADC();
  stopBtnPressed = !(PIND & (1 << PIN_STOPBTN_SW));
  wallPowerPresent = (PINB & (1 << PIN_PWR_SRC_SENSE));
  
  PCMSK2 |= (1 << PCINT19);  // PD3 - Stop button
  PCICR |= (1 << PCIE2);
  
  PCMSK0 |= (1 << PCINT7);   // PB7 - Power source sense
  PCICR |= (1 << PCIE0);
}

void SensorManager::setupPins() {
  DDRC &= ~(1 << PIN_I_NUC_SENSE);
  DDRA &= ~(1 << PIN_I_MOTORS_SENSE);
  DDRC &= ~(1 << PIN_V_SENSE);
  DDRC &= ~(1 << PIN_VBAT_SENSE);
  DDRD &= ~(1 << PIN_POWER_SW);
  DDRD &= ~(1 << PIN_STOPBTN_SW);
  DDRB &= ~(1 << PIN_PWR_SRC_SENSE);
}

void SensorManager::setupADC() {
  // Configure Timer1 for CTC mode to trigger ADC at 400 Hz
  // 16MHz / 64 (prescaler) / 625 (OCR1A) = 400 Hz
  TCCR1A = 0;
  TCCR1B = (1 << WGM12) | (1 << CS11) | (1 << CS10);  // CTC mode, prescaler 64
  OCR1A = 624;  // 16MHz / 64 / 625 = 400 Hz (OCR1A = count - 1)
  TIMSK1 = 0;   // No timer interrupt needed
  
  // Configure ADC: Timer1 Compare Match trigger, interrupt enabled
  ADCSRA = (1 << ADEN) | (1 << ADIE) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
  ADCSRB = (1 << ADTS2) | (1 << ADTS0);  // Trigger source: Timer1 Compare Match B
  ADCSRA |= (1 << ADATE);  // Auto-trigger enable
  ADMUX = (1 << REFS0) | (PIN_V_SENSE_ADC & 0x0F);
  
  delayMicroseconds(100);
  ADCSRA |= (1 << ADSC);  // Start first conversion
}

void SensorManager::selectADCChannel(uint8_t channel) {
  ADMUX = (ADMUX & 0xF0) | (channel & 0x0F);
}

SensorData SensorManager::getData() const {
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
      g_sensorManager->currentChannel = ADC_BATTERY_VOLTAGE;
      g_sensorManager->selectADCChannel(PIN_VBAT_SENSE_ADC);
      break;
      
    case ADC_BATTERY_VOLTAGE:
      g_sensorManager->batteryVoltage = adcValue;
      g_sensorManager->currentChannel = ADC_NUC_CURRENT;
      g_sensorManager->selectADCChannel(PIN_I_NUC_SENSE_ADC);
      break;
      
    case ADC_NUC_CURRENT:
      g_sensorManager->nucCurrent = adcValue;
      g_sensorManager->currentChannel = ADC_MOTOR_CURRENT;
      g_sensorManager->selectADCChannel(PIN_I_MOTORS_SENSE_ADC);
      break;
      
    case ADC_MOTOR_CURRENT:
      g_sensorManager->motorCurrent = adcValue;
      g_sensorManager->currentChannel = ADC_VOLTAGE;
      g_sensorManager->selectADCChannel(PIN_V_SENSE_ADC);
      break;
  }
  
  ADCSRA |= (1 << ADSC);
}

ISR(PCINT2_vect) {
  if (g_sensorManager != nullptr) {
    g_sensorManager->stopBtnPressed = !(PIND & (1 << PIN_STOPBTN_SW));
  }
}

ISR(PCINT0_vect) {
  if (g_sensorManager != nullptr) {
    bool wallPower = (PINB & (1 << PIN_PWR_SRC_SENSE));
    g_sensorManager->wallPowerPresent = wallPower;
    
    if (wallPower) {
      bitWrite(PORTC, PIN_MOTOR_PWR_CTRL, 0);
    }
  }
}