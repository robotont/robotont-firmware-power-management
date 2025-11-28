#include "MakitaBattery.h"
#include <avr/interrupt.h>

MakitaBattery::MakitaBattery(uint8_t enablePin, uint8_t dataPin)
  : enablePin(enablePin), dataPin(dataPin), oneWire(dataPin), debugCounter(0) {}

void MakitaBattery::begin() {
  pinMode(dataPin, OUTPUT);
  pinMode(enablePin, OUTPUT);
  disableBattery();
}

void MakitaBattery::update() {
  byte response[29];
  memset(response, 0, sizeof(response));
  
  enableBattery();
  
  oneWire.reset();
  delayMicroseconds(400);
  oneWire.write(0xCC, 0);
  delayMicroseconds(90);
  oneWire.write(0xD7, 0);
  delayMicroseconds(90);
  oneWire.write(0x00, 0);
  delayMicroseconds(90);
  oneWire.write(0x00, 0);
  delayMicroseconds(90);
  oneWire.write(0xFF, 0);
  
  // Read full response
  for (uint8_t i = 0; i < 29; i++) {
    delayMicroseconds(90);
    response[i] = oneWire.read();
  }

  disableBattery();
  
  // Parse response into data structure
  BatteryData& data = this->data;

  // Pack voltage (bytes 0-1)
  data.packVoltage = response[0] | (response[1] << 8);
  
  // Cell voltages (bytes 2-11, 5 cells × 2 bytes each)
  for (uint8_t i = 0; i < 5; i++) {
    data.cellVoltages[i] = response[2 + i*2] | (response[3 + i*2] << 8);
  }
  
  // Temperature sensors (bytes 16-19, 2 sensors × 2 bytes each)
  for (uint8_t i = 0; i < 2; i++) {
    data.temperatures[i] = response[14 + i*2] | (response[15 + i*2] << 8);
  }
}

void MakitaBattery::sendClearCommand() {
  oneWire.reset();
  delayMicroseconds(400);
  oneWire.write(0xCC, 0);
  delayMicroseconds(90);
  oneWire.write(0xF0, 0);
  delayMicroseconds(90);
  oneWire.write(0x00, 0);
}

void MakitaBattery::enableBattery() {
  digitalWrite(enablePin, HIGH);
  // Need to wait for the battery to wake up
  // 10 ms caused frequent read errors
  // 15 ms seems stable
  // using 20 ms to be more on a safe side
  delay(20);
}

void MakitaBattery::disableBattery() {
  //pinMode(enablePin, OUTPUT);
  digitalWrite(enablePin, LOW);
}

void MakitaBattery::sendCommand(const byte* cmd, uint8_t cmdLen, byte* response, uint8_t responseLen) {
  oneWire.reset();
  delayMicroseconds(400);
  
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