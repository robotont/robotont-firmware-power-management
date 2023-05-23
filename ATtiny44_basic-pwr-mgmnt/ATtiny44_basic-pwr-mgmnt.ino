//#include <avr/sleep.h>
//
//ISR(INT0_vect) {
//  
//}
#define POWER_ON PA0
#define MOTOR_ON PA1
#define ESTOP_SW PA2
#define ESTOP_SIG PA3
#define USB_SENSE PA4
#define ESTOP_LED_R PA5
#define ESTOP_LED_G PA6
#define BUZZER PA7

#define POWER_SW PB2

void setup() {
  DDRA = (1 << POWER_ON) | (1 << MOTOR_ON) | (1 << BUZZER) | (0 << ESTOP_SW) | (1 << ESTOP_SIG);

  DDRB = 0 << POWER_SW;
//  PORTB = 1 << POWER_SW;

  
}

byte prevButtonState = 0;
byte prevEstopState = 0;
unsigned long counter = 0;
byte systemOn = 0;

void loop() {
  byte buttonState = PINB & (1 << POWER_SW);
  byte estopState = PINA & (1 << ESTOP_SW);

  if (estopState) {
    // estop disable
    if (!prevEstopState) {
      digitalWrite(ESTOP_LED_R, HIGH);
      digitalWrite(ESTOP_LED_G, LOW);
    }


    if (prevButtonState && buttonState) counter = 0;
    // loe yles ainult siis kui nupp hoitakse all
    else if (!prevButtonState && !buttonState ) counter++;
  
    // sisselylitamine
    if (!systemOn) {
      if (counter > 300000) {
        PORTA = (1 << POWER_ON) | (0 << MOTOR_ON);  
        counter = 110000;
        systemOn = 1;
  
        for (int i = 0; i < 100; i++) {
          digitalWrite(BUZZER, HIGH);
          delayMicroseconds(1000);
          digitalWrite(BUZZER, LOW);
          delayMicroseconds(1000);
        }
        for (int i = 0; i < 100; i++) {
          digitalWrite(BUZZER, HIGH);
          delayMicroseconds(800);
          digitalWrite(BUZZER, LOW);
          delayMicroseconds(800);
        }
        for (int i = 0; i < 100; i++) {
          digitalWrite(BUZZER, HIGH);
          delayMicroseconds(600);
          digitalWrite(BUZZER, LOW);
          delayMicroseconds(600);
        }
      }
    }
  
    // systeem k2ib
    else {
      // kui nupp lastakse lahti
      if (!prevButtonState && buttonState) {
        // kui mootorid pole sees
        if (!(PORTA & (1 << MOTOR_ON))) {
          // kui vajutati korraks
          if (counter < 100000 && counter > 5000) {
            PORTA = (1 << MOTOR_ON) | (1 << POWER_ON); 
          }
        }
        // kui mootorid on sees
        else if (PORTA & (1 << MOTOR_ON)) {
          // kui vajutati korraks
          if (counter < 100000 && counter > 5000) {
            PORTA = (0 << MOTOR_ON) | (1 << POWER_ON); 
          }
        }
        counter = 0;
      }
      // nuppu hoitakse all
      if (counter > 400000) {
        PORTA = (0 << POWER_ON) | (0 << MOTOR_ON);  
        counter = 0;
        systemOn = 0;
        for (int i = 0; i < 100; i++) {
          digitalWrite(BUZZER, HIGH);
          delayMicroseconds(600);
          digitalWrite(BUZZER, LOW);
          delayMicroseconds(600);
        }
        for (int i = 0; i < 100; i++) {
          digitalWrite(BUZZER, HIGH);
          delayMicroseconds(800);
          digitalWrite(BUZZER, LOW);
          delayMicroseconds(800);
        }
        for (int i = 0; i < 100; i++) {
          digitalWrite(BUZZER, HIGH);
          delayMicroseconds(1000);
          digitalWrite(BUZZER, LOW);
          delayMicroseconds(1000);
        }
      }
    }
  }
  else {
    // estop enable
    if (prevEstopState) {
      digitalWrite(MOTOR_ON, LOW);
      digitalWrite(ESTOP_LED_G, HIGH);
      digitalWrite(ESTOP_LED_R, LOW);
    }

  }
  prevButtonState = buttonState;
  prevEstopState = estopState;
} 
