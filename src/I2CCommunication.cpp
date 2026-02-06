#include "I2CCommunication.h"
#include <util/atomic.h>

I2CCommunication::I2CCommunication() {}

void I2CCommunication::begin() {
  initTWI();
}

void I2CCommunication::initTWI() {
  // Disable TWI first
  TWCR = 0;
  
  // Set bitrate: 100kHz @ 8MHz CPU
  // SCL_freq = CPU_freq / (16 + 2*TWBR*prescaler)
  // 100000 = 8000000 / (16 + 2*32*1) => TWBR=32, prescaler=1
  TWSR = 0x00;  // Prescaler = 1
  TWBR = 32;
  
  // Enable TWI
  TWCR = (1 << TWEN);
}

//=============================================================================
// Open-drain I2C bit-banging helpers
// To simulate open-drain on AVR:
//   LOW:     DDR=1 (output), PORT=0 (drive low)
//   RELEASE: DDR=0 (input),  PORT=0 (let pull-up pull high)
//=============================================================================

inline void I2CCommunication::sdaLow() {
  SDA_PORT &= ~SDA_MASK;  // Ensure PORT bit is 0
  SDA_DDR  |= SDA_MASK;   // Set as output (drives low)
}

inline void I2CCommunication::sdaRelease() {
  SDA_DDR  &= ~SDA_MASK;  // Set as input (high-Z)
  SDA_PORT &= ~SDA_MASK;  // No internal pull-up (external pull-up required)
}

inline void I2CCommunication::sclLow() {
  SCL_PORT &= ~SCL_MASK;
  SCL_DDR  |= SCL_MASK;
}

inline void I2CCommunication::sclRelease() {
  SCL_DDR  &= ~SCL_MASK;
  SCL_PORT &= ~SCL_MASK;
}

inline bool I2CCommunication::sdaRead() {
  return (SDA_PIN & SDA_MASK) != 0;
}

inline bool I2CCommunication::sclRead() {
  return (SCL_PIN & SCL_MASK) != 0;
}

// Wait for SCL to go high (handles clock stretching)
bool I2CCommunication::waitSclHigh(uint16_t timeoutUs) {
  while (timeoutUs--) {
    if (sclRead()) return true;
    delayMicroseconds(1);
  }
  return false;
}

bool I2CCommunication::busStuck() {
  // Temporarily disable TWI to read pin states
  uint8_t twcr_save = TWCR;
  TWCR = 0;
  
  // Release both lines
  sdaRelease();
  sclRelease();
  delayMicroseconds(10);
  
  bool stuck = !sdaRead() || !sclRead();
  
  // Restore TWI
  TWCR = twcr_save;
  return stuck;
}

void I2CCommunication::resetBus() {
  // Disable TWI hardware - releases pins to GPIO control
  TWCR = 0;
  
  // Release both lines first
  sdaRelease();
  sclRelease();
  delayMicroseconds(10);
  
  //===========================================================================
  // Phase 1: Handle SCL stuck low (slave clock-stretching indefinitely)
  //===========================================================================
  if (!sclRead()) {
    // SCL is held low by slave - this is bad, not much we can do
    // Wait a bit and hope slave releases
    for (uint8_t i = 0; i < 100; i++) {
      delayMicroseconds(100);
      if (sclRead()) break;
    }
    // If still stuck, we'll try clocking anyway but it probably won't work
  }
  
  //===========================================================================
  // Phase 2: Clock out stuck slave (SDA held low)
  // A slave might be in the middle of sending a byte and waiting for clocks.
  // We send up to 9 clocks (max bits in a byte + ACK) to let it finish.
  //===========================================================================
  for (uint8_t i = 0; i < 9; i++) {
    // Check if SDA is released
    if (sdaRead()) {
      break;  // Bus is free!
    }
    
    // Generate one clock pulse
    sclLow();
    delayMicroseconds(5);
    sclRelease();
    
    // Wait for SCL to actually go high (with timeout for clock stretching)
    if (!waitSclHigh(1000)) {
      // SCL stuck low - slave is clock stretching, wait longer
      delayMicroseconds(100);
    }
    delayMicroseconds(5);
  }
  
  //===========================================================================
  // Phase 3: Generate a STOP condition to reset slave state machines
  // STOP = SDA rising edge while SCL is high
  //===========================================================================
  
  // Ensure SCL is low first
  sclLow();
  delayMicroseconds(5);
  
  // Drive SDA low
  sdaLow();
  delayMicroseconds(5);
  
  // Release SCL (goes high via pull-up)
  sclRelease();
  if (!waitSclHigh(1000)) {
    // SCL won't go high - bus is seriously stuck
  }
  delayMicroseconds(5);
  
  // Release SDA while SCL is high = STOP condition
  sdaRelease();
  delayMicroseconds(10);
  
  //===========================================================================
  // Phase 4: Verify bus is free
  //===========================================================================
  if (!sdaRead() || !sclRead()) {
    // Bus still stuck - one more aggressive attempt
    // Generate multiple STOPs
    for (uint8_t i = 0; i < 3; i++) {
      sclLow();
      sdaLow();
      delayMicroseconds(10);
      sclRelease();
      delayMicroseconds(10);
      sdaRelease();
      delayMicroseconds(10);
    }
  }
  
  // Small delay before re-enabling TWI
  delayMicroseconds(50);
  
  // Re-enable TWI hardware
  initTWI();
}

bool I2CCommunication::attemptTransmission(const byte* data, size_t length) {
  uint16_t timeout;
  
  // Check if bus is stuck before even trying
  if (busStuck()) {
    return false;
  }
  
  // Send START
  TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
  timeout = 10000;
  while (!(TWCR & (1 << TWINT)) && --timeout);
  if (!timeout) return false;
  
  uint8_t status = TWSR & 0xF8;
  if (status != 0x08 && status != 0x10) {  // START or repeated START
    // Send STOP to abort
    TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
    return false;
  }
  
  // Send address + write
  TWDR = (I2C_SLAVE_ADDRESS << 1) | 0;
  TWCR = (1 << TWINT) | (1 << TWEN);
  timeout = 10000;
  while (!(TWCR & (1 << TWINT)) && --timeout);
  if (!timeout || (TWSR & 0xF8) != 0x18) {  // SLA+W ACK
    TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
    return false;
  }
  
  // Send data bytes
  for (size_t i = 0; i < length; i++) {
    TWDR = data[i];
    TWCR = (1 << TWINT) | (1 << TWEN);
    timeout = 10000;
    while (!(TWCR & (1 << TWINT)) && --timeout);
    if (!timeout || (TWSR & 0xF8) != 0x28) {  // Data ACK
      TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
      return false;
    }
  }
  
  // Send STOP
  TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
  
  // Wait for STOP to complete (TWSTO clears automatically)
  timeout = 1000;
  while ((TWCR & (1 << TWSTO)) && --timeout);
  
  return true;
}

void I2CCommunication::sendData(const SensorData& sensorData, const BatteryData& batteryData) {
  byte dataPacket[25];
  
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    dataPacket[0] = sensorData.status;
    dataPacket[1] = highByte(sensorData.motorCurrent);
    dataPacket[2] = lowByte(sensorData.motorCurrent);
    dataPacket[3] = highByte(sensorData.nucCurrent);
    dataPacket[4] = lowByte(sensorData.nucCurrent);
    dataPacket[5] = highByte(sensorData.voltage);
    dataPacket[6] = lowByte(sensorData.voltage);
    dataPacket[7] = highByte(sensorData.batteryVoltage);
    dataPacket[8] = lowByte(sensorData.batteryVoltage);
    
    dataPacket[9]  = highByte(batteryData.packVoltage);
    dataPacket[10] = lowByte(batteryData.packVoltage);
    for (uint8_t i = 0; i < 5; i++) {
      dataPacket[11 + i * 2] = highByte(batteryData.cellVoltages[i]);
      dataPacket[12 + i * 2] = lowByte(batteryData.cellVoltages[i]);
    }
    
    for (uint8_t i = 0; i < 2; i++) {
      dataPacket[21 + i * 2] = highByte(batteryData.temperatures[i]);
      dataPacket[22 + i * 2] = lowByte(batteryData.temperatures[i]);
    }
  }

  for (uint8_t attempt = 0; attempt < MAX_RETRY_ATTEMPTS; attempt++) {
    if (attemptTransmission(dataPacket, sizeof(dataPacket))) {
      return;  // Success
    }
    
    // Failed - reset bus and retry
    resetBus();
    delay(10);  // Short delay between retries
  }
  
  // All attempts failed - bus might be completely dead
  // Could add error reporting here
}