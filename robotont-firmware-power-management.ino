
#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdio.h>

//IF POSSIBLE DONT USE pinMode() function, some pin mappings are not valid


//GATES
#define PIN_BAT_PWR_CTRL 7 // PortB
#define PIN_SYS_PWR_CTRL 3 //PortA
#define PIN_MOTOR_PWR_CTRL 7 //PortC
#define PIN_WALL_PWR_CTRL 4 //PortD
//Current sensing pin numbers
#define PIN_I_MOTORS_SENSE 1 //PORTA
#define PIN_I_NUC_SENSE 3 //PORTC
//Voltage sensing pin numbers
#define PIN_V_SENSE 1 //PORTC
#define PIN_VBAT_SENSE 2 //PORTC
#define PIN_POWER_SW 2 //PORTD
#define PIN_ESTOP_SW 3 //PORTD
#define PIN_DBG_LED_G 6 //PORTD
#define PIN_DBG_LED_R 5 //PORTD
#define PIN_BUZZER 0 //PD0

//Defines for analogRead function
#define V_SENSE PC1
#define VBAT_SENSE PC2
#define I_MOTORS_SENSE PA1
#define I_NUC_SENSE PC3

#define POWER_SW PD2
#define ESTOP_SW PD3

enum PowerState{
  POWER_OFF,
  POWER_ON
};

enum SwitchingState{
  INIT,
  CONNECTED_TO_WALL,
  CONNECTED_TO_BAT
};

uint8_t toggle=0;

void setup() {
  
  //Current Sensing pins as INPUT
  DDRC &= ~(1 << PIN_I_NUC_SENSE);
  DDRA &= ~(1 << PIN_I_MOTORS_SENSE);

  //Voltage Sensing pins as INPUT
  DDRC &= ~(1 << PIN_V_SENSE);
  DDRC &= ~(1 << PIN_VBAT_SENSE);

  //POWER BUTTON as INPUT
  DDRD &= ~(1 << PIN_POWER_SW);
  //ESTOP as INPUT
  DDRD &= ~(1 << PIN_ESTOP_SW);
  PCMSK2 |= (1 << PCINT19);   //Enable Pin Change Interrupt
  PCICR |= (1 << PCIE2); // Enable Pin Change Interrupt for Port D

  //LEDS as OUTPUT
  DDRD |= (1 << PIN_DBG_LED_R);
  DDRD |= (1 << PIN_DBG_LED_G);
  bitWrite(PORTD, PIN_DBG_LED_R, 0);
  bitWrite(PORTD, PIN_DBG_LED_G, 0);

  //GATES as OUTPUT
  DDRB |= (1 << PIN_BAT_PWR_CTRL);
  DDRD |= (1 << PIN_WALL_PWR_CTRL);
  DDRC |= (1 << PIN_MOTOR_PWR_CTRL);
  DDRA |= (1 << PIN_SYS_PWR_CTRL);

  //ALL GATES CLOSED
  bitWrite(PORTB, PIN_BAT_PWR_CTRL, 0);
  bitWrite(PORTD, PIN_WALL_PWR_CTRL, 0);
  bitWrite(PORTC, PIN_MOTOR_PWR_CTRL, 0);
  bitWrite(PORTA, PIN_SYS_PWR_CTRL, 0);
  
  const int interruptFrequency = 1500;
  TCCR0A = (1<<CTC0) | (1<<CS01); //CTC and 8 prescaler
  TCNT0  = 0; //Counter to zero
  OCR0A = 4000000 / (2 *8 *interruptFrequency) - 1; 
  // Enable Timer/Counter0 Output Compare Match A Interrupt Enable
  TIMSK0 |= (1 << OCIE0A);

  ADCSRA |= (0 << ADPS2) | (0 << ADPS1) | (0 << ADPS0); //ADC prescaler to minimal //Not tested

  sei();

}

enum PowerState powerState = POWER_OFF;
enum SwitchingState switchingState = INIT;
uint8_t EStopPressed = 0;

void loop() {
  
  bitWrite(PORTD, PIN_DBG_LED_G, 1);
  delay(10000);
  bitWrite(PORTD, PIN_DBG_LED_G, 0);
  delay(10000);

  if (!digitalRead(POWER_SW))
  {
    if (powerState == POWER_OFF)
    {
      bitWrite(PORTD, PIN_DBG_LED_R, 1);
      powerState = POWER_ON;
    }
    else
    {
      // Turn power off
      powerState = POWER_OFF;
      switchingState = INIT;
      bitWrite(PORTD, PIN_DBG_LED_R, 0);
      bitWrite(PORTB, PIN_BAT_PWR_CTRL, 0);
      bitWrite(PORTD, PIN_WALL_PWR_CTRL, 0);
      bitWrite(PORTC, PIN_MOTOR_PWR_CTRL, 0);
      bitWrite(PORTA, PIN_SYS_PWR_CTRL, 0);
    }
  }
}

//Bat off, wall on
void switchBatToWall(){
  bitWrite(PORTA, PIN_SYS_PWR_CTRL, 1); //SYS ON
  bitWrite(PORTC, PIN_MOTOR_PWR_CTRL, !EStopPressed); //MOTORS OFF
  bitWrite(PORTB, PIN_BAT_PWR_CTRL, 0);
  bitWrite(PORTD, PIN_WALL_PWR_CTRL, 1);
 
  switchingState = CONNECTED_TO_WALL;
}

//Wall off, bat on
void switchWallToBat(){
  bitWrite(PORTA, PIN_SYS_PWR_CTRL, 1); //SYS ON
  bitWrite(PORTC, PIN_MOTOR_PWR_CTRL, !EStopPressed); //MOTORS ON if ESTOP not pressed
  bitWrite(PORTD, PIN_WALL_PWR_CTRL, 0);
  bitWrite(PORTB, PIN_BAT_PWR_CTRL, 1);
  
  switchingState = CONNECTED_TO_BAT;
}

ISR(TIMER0_COMPA_vect) {
  
  if(powerState == POWER_ON){
    bitWrite(PORTD, PIN_BUZZER, !toggle);
    uint16_t V = analogRead(V_SENSE);
    uint16_t VBAT = analogRead(VBAT_SENSE);
    if(V >= VBAT && (switchingState !=  CONNECTED_TO_BAT)){
      switchBatToWall();
    }
    else if(V < VBAT && (switchingState != CONNECTED_TO_WALL)){
      switchWallToBat();
    }
  }
}


ISR(PCINT2_vect) {
    bitWrite(PORTD, PIN_DBG_LED_R, 1);
    if(!digitalRead(ESTOP_SW)){ //Button 0 when pressed
      EStopPressed=1;
      bitWrite(PORTC, PIN_MOTOR_PWR_CTRL, 0);
    }
    else{
      EStopPressed=0;
      if(switchingState == CONNECTED_TO_BAT){
        bitWrite(PORTC, PIN_MOTOR_PWR_CTRL, 1);
      }
    }
}

//Current calculations:
uint16_t measureMotorCurrent(){
  //Shunt 5m, Gain 40x
  // V = (analogRead(I_MOTORS_SENSE) * 3.3)/(1024*40)
  // I = V / Shunt  ---  I = V * (1/Shunt)
  //((analogRead(I_MOTORS_SENSE) * 3.3)/1024*40)*200
  uint16_t MtrCurrent=analogRead(I_MOTORS_SENSE)/62.061; //Amp
  
  return MtrCurrent; 
}
//0.673 - 1.13a
uint16_t measureNUCCurrent(){
  //Shunt 10m, Gain 60x
  //V = (analogRead(I_MOTORS_SENSE) * 3.3)/(1024*60) //60x gain
  // I = V / Shunt  ---  I = V * (1/Shunt)
  //((analogRead(I_NUC_SENSE) * 3.3)/1024*60)*100
  uint16_t NucCurrent=analogRead(I_NUC_SENSE)/186.182; //Amp
  
  return NucCurrent; 
}
