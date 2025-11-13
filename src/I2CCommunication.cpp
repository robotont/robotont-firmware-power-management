#include "I2CCommunication.h"
#include <util/atomic.h>

I2CCommunication::I2CCommunication() {}

void I2CCommunication::begin() {
  Wire.begin();
}

void I2CCommunication::sendData(const SensorData& sensorData, const BatteryDebugData& batteryData) {
  byte dataPacket[16];
  
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    dataPacket[0] = highByte(sensorData.motorCurrent);
    dataPacket[1] = lowByte(sensorData.motorCurrent);
    dataPacket[2] = highByte(sensorData.nucCurrent);
    dataPacket[3] = lowByte(sensorData.nucCurrent);
    dataPacket[4] = highByte(sensorData.voltage);
    dataPacket[5] = lowByte(sensorData.voltage);
    dataPacket[6] = highByte(sensorData.batteryVoltage);
    dataPacket[7] = lowByte(sensorData.batteryVoltage);
    
    for (uint8_t i = 0; i < 8; i++) {
      dataPacket[8 + i] = batteryData.data[i];
    }
  }
  
  for (uint8_t attempt = 0; attempt < MAX_RETRY_ATTEMPTS; attempt++) {
    if (attemptTransmission(dataPacket, sizeof(dataPacket))) {
      return;
    }
    recoverBus();
    delay(10);
  }
}

bool I2CCommunication::attemptTransmission(const byte* data, size_t length) {
  Wire.beginTransmission(I2C_SLAVE_ADDRESS);
  Wire.write(data, length);
  return Wire.endTransmission() == 0;
}

void I2CCommunication::recoverBus() {
  TWCR = 0;
  
  SCL_DDR |= SCL_MASK;
  SCL_PORT |= SCL_MASK;
  SDA_DDR &= ~SDA_MASK;
  
  for (uint8_t i = 0; i < 9; i++) {
    SCL_PORT &= ~SCL_MASK;
    delayMicroseconds(5);
    SCL_PORT |= SCL_MASK;
    delayMicroseconds(5);
  }
  
  SDA_DDR |= SDA_MASK;
  SDA_PORT &= ~SDA_MASK;
  delayMicroseconds(5);
  SCL_PORT |= SCL_MASK;
  delayMicroseconds(5);
  SDA_PORT |= SDA_MASK;
  delayMicroseconds(5);
  
  SDA_DDR &= ~SDA_MASK;
  SCL_DDR &= ~SCL_MASK;
  
  Wire.begin();
}