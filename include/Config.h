#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// Disclaimer
// Do not use arduino bitWrite/bitRead/bitSet/bitClear macros for
// manipulating hardware registers. They bitshift 32-bit integers, which
// can lead to unexpected results on 8-bit attiny88 registers. Use direct bitwise
// operations instead for example:
    // PORTA |= (1 << PIN_SYS_PWR_CTRL);   // set output to HIGH
    // PORTA &= ~(1 << PIN_SYS_PWR_CTRL);  // set output to LOW


// Pin Definitions
#define PIN_SYS_PWR_CTRL 3       // PA3
#define PIN_MOTOR_PWR_CTRL 7     // PC7

#define PIN_I_MOTORS_SENSE 1     // PA1
#define PIN_I_NUC_SENSE 3        // PC3
#define PIN_V_SENSE 1            // PC1
#define PIN_VBAT_SENSE 2         // PC2

#define PIN_POWER_SW 2           // PD2
#define PIN_STOPBTN_SW 3         // PD3
#define PIN_DBG_LED_G 5          // PD5
#define PIN_DBG_LED_R 6          // PD6
#define PIN_STOPBTN_LED_R 1      // PB1
#define PIN_STOPBTN_LED_G 2      // PB2
#define PIN_BUZZER 0             // PD0

#define PIN_V_SENSE_ADC 1        // PC1 (ADC1)
#define PIN_VBAT_SENSE_ADC 2     // PC2 (ADC2)
#define PIN_I_MOTORS_SENSE_ADC 7 // PA1 (ADC7)
#define PIN_I_NUC_SENSE_ADC 3    // PC3 (ADC3)
#define PIN_PWR_SRC_SENSE 7      // PB7

#define PIN_BAT_EN 7             // PD7, arduino pin 7
#define PIN_BAT_DATA 8           // PB0, arduino pin 8

// Timing Constants
#define POWER_ON_HOLD_TIME 600
#define POWER_OFF_HOLD_TIME 1200

// I2C Configuration
#define I2C_SLAVE_ADDRESS 0x12

#define SDA_DDR   DDRC
#define SDA_PORT  PORTC
#define SDA_PIN   PINC      // ADD THIS - for reading pin state
#define SDA_MASK  (1 << PC4)

#define SCL_DDR   DDRC
#define SCL_PORT  PORTC
#define SCL_PIN   PINC      // ADD THIS
#define SCL_MASK  (1 << PC5)

// Audio Frequencies (Hz)
#define BEEP_FREQ_HIGH 1000
#define BEEP_FREQ_MED 800
#define BEEP_FREQ_LOW 500

#endif