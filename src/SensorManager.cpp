#include "SensorManager.h"
#include <avr/interrupt.h>
#include <util/delay.h>

SensorManager* g_sensorManager = nullptr;

SensorManager::SensorManager()
  : status(0), voltage(0), batteryVoltage(0), nucCurrent(0), motorCurrent(0),
    currentChannel(ADC_VOLTAGE) {
  g_sensorManager = this;
}

void SensorManager::begin() {
  setupPins();
  setupADC();
  
  // Read initial input states
  updateInputStatus();
  
  // Setup pin change interrupts for stop button and wall power
  PCMSK2 |= (1 << PCINT19);  // PD3 - Stop button
  PCICR |= (1 << PCIE2);
  
  PCMSK0 |= (1 << PCINT7);   // PB7 - Power source sense
  PCICR |= (1 << PCIE0);
}

void SensorManager::setupPins() {
  // ADC inputs
  DDRC &= ~(1 << PIN_I_NUC_SENSE);
  DDRA &= ~(1 << PIN_I_MOTORS_SENSE);
  DDRC &= ~(1 << PIN_V_SENSE);
  DDRC &= ~(1 << PIN_VBAT_SENSE);
  
  // Digital inputs
  DDRD &= ~(1 << PIN_POWER_SW);
  DDRD &= ~(1 << PIN_STOPBTN_SW);
  DDRB &= ~(1 << PIN_PWR_SRC_SENSE);
}

void SensorManager::setupADC() {
  // Configure Timer1 for CTC mode to trigger ADC at 400 Hz
  TCCR1A = 0;
  TCCR1B = (1 << WGM12) | (1 << CS11) | (1 << CS10);  // CTC mode, prescaler 64
  OCR1A = 624;  // 16MHz / 64 / 625 = 400 Hz
  TIMSK1 = 0;
  
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

void SensorManager::updateInputStatus() {
  bool stopBtn = !(PIND & (1 << PIN_STOPBTN_SW));
  bool powerSw = (PIND & (1 << PIN_POWER_SW)) != 0;
  bool wallPower = (PINB & (1 << PIN_PWR_SRC_SENSE)) != 0;
  
  uint8_t newStatus = status;
  
  if (stopBtn)   newStatus |= Status::STOP_BTN_MASK;
  else           newStatus &= ~Status::STOP_BTN_MASK;
  
  if (powerSw)   newStatus |= Status::POWER_SW_MASK;
  else           newStatus &= ~Status::POWER_SW_MASK;
  
  if (wallPower) newStatus |= Status::WALL_POWER_MASK;
  else           newStatus &= ~Status::WALL_POWER_MASK;
  
  status = newStatus;
}

void SensorManager::setMotorPower(bool on) {
  if (on) status |= Status::MOTOR_POWER_MASK;
  else    status &= ~Status::MOTOR_POWER_MASK;
}

void SensorManager::setSysPower(bool on) {
  if (on) status |= Status::SYS_POWER_MASK;
  else    status &= ~Status::SYS_POWER_MASK;
}

uint8_t SensorManager::getStatus() const {
  uint8_t s;
  cli();
  s = status;
  sei();
  return s;
}

SensorData SensorManager::getData() const {
  SensorData data;
  cli();
  data.status = status;
  data.voltage = voltage;
  data.batteryVoltage = batteryVoltage;
  data.nucCurrent = nucCurrent;
  data.motorCurrent = motorCurrent;
  sei();
  return data;
}

// ADC conversion complete ISR
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

// Stop button pin change ISR
ISR(PCINT2_vect) {
  if (g_sensorManager == nullptr) return;
  
  bool stopBtn = !(PIND & (1 << PIN_STOPBTN_SW));
  if (stopBtn) g_sensorManager->status |= Status::STOP_BTN_MASK;
  else         g_sensorManager->status &= ~Status::STOP_BTN_MASK;
}

// Wall power pin change ISR
ISR(PCINT0_vect) {
  if (g_sensorManager == nullptr) return;
  
  bool wallPower = (PINB & (1 << PIN_PWR_SRC_SENSE)) != 0;
  if (wallPower) {
    g_sensorManager->status |= Status::WALL_POWER_MASK;
    // Immediate motor shutoff on wall power connect (safety)
    PORTC &= ~(1 << PIN_MOTOR_PWR_CTRL);
    g_sensorManager->status &= ~Status::MOTOR_POWER_MASK;
  } else {
    g_sensorManager->status &= ~Status::WALL_POWER_MASK;
  }
}