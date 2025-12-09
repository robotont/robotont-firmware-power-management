#include "I2CCommunication.h"
#include <util/atomic.h>

I2CCommunication::I2CCommunication() {}

void I2CCommunication::begin() {
  Wire.begin();
}

void I2CCommunication::sendData(const SensorData& sensorData, const BatteryData& batteryData) {
  byte dataPacket[25];
  
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    // Byte 0: Status byte (contains all state flags)
    dataPacket[0] = sensorData.status;
    
    // Bytes 1-8: Analog sensor data
    dataPacket[1] = highByte(sensorData.motorCurrent);
    dataPacket[2] = lowByte(sensorData.motorCurrent);
    dataPacket[3] = highByte(sensorData.nucCurrent);
    dataPacket[4] = lowByte(sensorData.nucCurrent);
    dataPacket[5] = highByte(sensorData.voltage);
    dataPacket[6] = lowByte(sensorData.voltage);
    dataPacket[7] = highByte(sensorData.batteryVoltage);
    dataPacket[8] = lowByte(sensorData.batteryVoltage);
    
    // Bytes 9-20: Battery pack data
    dataPacket[9]  = highByte(batteryData.packVoltage);
    dataPacket[10] = lowByte(batteryData.packVoltage);
    for (uint8_t i = 0; i < 5; i++) {
      dataPacket[11 + i * 2] = highByte(batteryData.cellVoltages[i]);
      dataPacket[12 + i * 2] = lowByte(batteryData.cellVoltages[i]);
    }
    
    // Bytes 21-24: Temperature data
    for (uint8_t i = 0; i < 2; i++) {
      dataPacket[21 + i * 2] = highByte(batteryData.temperatures[i]);
      dataPacket[22 + i * 2] = lowByte(batteryData.temperatures[i]);
    }
  }

  for (uint8_t attempt = 0; attempt < MAX_RETRY_ATTEMPTS; attempt++) {
    if (attemptTransmission(dataPacket, sizeof(dataPacket))) {
      return;
    }
    recoverBus();
    delay(500);
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