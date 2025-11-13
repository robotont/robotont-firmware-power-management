#include "MakitaBattery.h"
#include <avr/interrupt.h>

MakitaBattery::MakitaBattery(uint8_t enablePin, uint8_t dataPin)
  : enablePin(enablePin), dataPin(dataPin), oneWire(dataPin), debugCounter(0) {}

void MakitaBattery::begin() {
  pinMode(enablePin, OUTPUT);
  pinMode(dataPin, OUTPUT);
  disableBattery();
}

void MakitaBattery::update() {
  byte response[8];
  memset(response, 0, sizeof(response));
  
  disableInterrupts();
  enableBattery();
  
  const byte clearCmd[] = {BATTERY_CLEAR_CMD_0, BATTERY_CLEAR_CMD_1};
  sendCommand(clearCmd, sizeof(clearCmd), nullptr, 0);
  
  disableBattery();
  enableInterrupts();
  
  debugData.data[0] = response[0];
  debugData.data[1] = response[1];
  debugData.data[7] = debugCounter++;
}

void MakitaBattery::enableBattery() {
  digitalWrite(enablePin, HIGH);
  delayMicroseconds(500);
  digitalWrite(enablePin, LOW);
  delayMicroseconds(500);
  digitalWrite(enablePin, HIGH);
  delayMicroseconds(500);
}

void MakitaBattery::disableBattery() {
  digitalWrite(enablePin, LOW);
}

void MakitaBattery::sendCommand(const byte* cmd, uint8_t cmdLen, byte* response, uint8_t responseLen) {
  oneWire.reset();
  delayMicroseconds(400);
  oneWire.write(BATTERY_SKIP_ROM_CMD, 0);
  
  for (uint8_t i = 0; i < cmdLen; i++) {
    delayMicroseconds(90);
    oneWire.write(cmd[i], 0);
  }
  
  if (response != nullptr && responseLen > 0) {
    for (uint8_t i = 0; i < responseLen; i++) {
      delayMicroseconds(90);
      response[i] = oneWire.read();
    }
  }
}

void MakitaBattery::disableInterrupts() {
  cli();
  ADCSRA = 0;
  ADCSRA |= (1 << ADIF);
  sei();
}

void MakitaBattery::enableInterrupts() {
  cli();
  ADCSRA = (1 << ADEN) | (1 << ADIE) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
  sei();
}