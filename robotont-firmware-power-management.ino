
#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdio.h>
#include <Wire.h>

//IF POSSIBLE DONT USE pinMode() function, some pin mappings are not valid


//GATES
#define PIN_BAT_PWR_CTRL 7 // PortB
#define PIN_SYS_PWR_CTRL 3 //PortA
#define PIN_MOTOR_PWR_CTRL 7 //PortC
#define PIN_WALL_PWR_CTRL 4 //PortD
//Current sensing pin numbers
#define PIN_I_MOTORS_SENSE 1 //PORTA //ADC channel channel7
#define PIN_I_NUC_SENSE 3 //PORTC //ADC channel channel 3
//Voltage sensing pin numbers
#define PIN_V_SENSE 1 //PORTC //ADC channel 1
#define PIN_VBAT_SENSE 2 //PORTC //ADC channel 2
#define PIN_POWER_SW 2 //PORTD
#define PIN_ESTOP_SW 3 //PORTD
#define PIN_DBG_LED_G 6 //PORTD
#define PIN_DBG_LED_R 5 //PORTD
#define ESTOP_LED_R 1 //PORTB
#define ESTOP_LED_G 2 //PORTG
#define PIN_BUZZER 0 //PD0

//Defines for analogRead function
#define V_SENSE_ADC 1
#define VBAT_SENSE_ADC 2
#define I_MOTORS_SENSE_ADC 7
#define I_NUC_SENSE_ADC 3

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

enum PowerState powerState = POWER_OFF;
enum SwitchingState switchingState = INIT;
uint8_t EStopPressed = 0;

uint16_t V;
uint16_t VBAT;

void setup() {
  
  //Current Sensing pins as INPUT
  DDRC &= ~(1 << PIN_I_NUC_SENSE);
  DDRA &= ~(1 << PIN_I_MOTORS_SENSE);

  //Voltage Sensing pins as INPUT
  DDRC &= ~(1 << PIN_V_SENSE);
  DDRC &= ~(1 << PIN_VBAT_SENSE);

  //POWER BUTTON as INPUT
  DDRD &= ~(1 << PIN_POWER_SW);
  //ESTOP SWITCH as INPUT
  DDRD &= ~(1 << PIN_ESTOP_SW);
  PCMSK2 |= (1 << PCINT19);   //Enable Pin Change Interrupt
  PCICR |= (1 << PCIE2); // Enable Pin Change Interrupt for Port D

  //LEDS as OUTPUT
  DDRD |= (1 << PIN_DBG_LED_R);
  DDRD |= (1 << PIN_DBG_LED_G);
  bitWrite(PORTD, PIN_DBG_LED_R, 0);
  bitWrite(PORTD, PIN_DBG_LED_G, 0);

  //ESTOP LEDS as OUTPUT
  DDRB |= (1 << ESTOP_LED_R);
  DDRB |= (1 << ESTOP_LED_G);
  bitWrite(PORTB, ESTOP_LED_R, 1);
  bitWrite(PORTB, ESTOP_LED_G, 1);

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

  //BUZZER as output
  DDRD |= (1 << PIN_BUZZER);
  bitWrite(PORTB, PIN_BAT_PWR_CTRL, 0);

  //Initial ESTOP value
  if(!digitalRead(ESTOP_SW)){
    EStopPressed = 1;
  }
  
  const int interruptFrequency = 1500; //Higher freq does not work
  TCCR0A = (1<<CTC0) | (1<<CS01); //CTC and 2 prescaler
  TCNT0  = 0; //Counter to zero
  OCR0A = 1000000 / (2 *2 *interruptFrequency) - 1; 
  TIMSK0 |= (1 << OCIE0A); // Enable Timer/Counter0 Output Compare Match A Interrupt Enable

  
  ADCSRA |= (1 << ADEN) | (0 << ADPS2) | (0 << ADPS1) | (1 << ADPS0);
  ADCSRB |= (0 << ADTS2) | (0 << ADTS1) | (0 << ADTS0);
  ADMUX |= (1 << REFS0) | (2 & 0x0F);
  //ADCSRA |= (1 << ADSC); // Start conversion
  //ADCSRA |= (1 << ADATE);
  Wire.begin(); // join i2c bus as master
  
  sei(); //Enable interrupts

}


void loop() {
  
  bitWrite(PORTD, PIN_DBG_LED_G, 1);
  delay(10000);
  bitWrite(PORTD, PIN_DBG_LED_G, 0);
  delay(10000);
  sendDataOverI2C();

  if(switchingState==CONNECTED_TO_BAT && VBAT<680){
    beep(1000);
  }

  if (!digitalRead(POWER_SW))
  {
    if (powerState == POWER_OFF)
    {
      bitWrite(PORTD, PIN_DBG_LED_R, 1);
      powerState = POWER_ON;
      // Buzzer output that system is ON
      beep(1000);
      beep(800);
      beep(600);
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
      //Buzzer output that system is OFF
      beep(600);
      beep(800);
      beep(1000);
    }
  }
}


//Custom analogRead, takes ADC channel as input and returns the measured value
uint16_t ADCRead(uint8_t channel){
  
  ADMUX = (ADMUX & 0xF0) | (channel & 0x0F); //Select adc channel
  ADCSRA |= (1 << ADSC); // Start conversion
  // Wait for conversion to complete
  while (!(ADCSRA & (1 << ADIF)));

  uint8_t low = ADCL;
  uint8_t high = ADCH;
  
  return (high << 8) | low; // Combine low and high
}

void beep(uint16_t note)
{
  for (uint8_t i = 0; i < 100; i++)
  {
    bitWrite(PORTD, PIN_BUZZER, HIGH);
    delayMicroseconds(note);
    bitWrite(PORTD, PIN_BUZZER, LOW);
    delayMicroseconds(note);
  }
}

//Bat off, wall on
void switchBatToWall(){
  bitWrite(PORTA, PIN_SYS_PWR_CTRL, 1); //SYS ON
  bitWrite(PORTC, PIN_MOTOR_PWR_CTRL, 0); //MOTORS OFF
  bitWrite(PORTB, PIN_BAT_PWR_CTRL, 0);
  bitWrite(PORTD, PIN_WALL_PWR_CTRL, 1);
 
  switchingState = CONNECTED_TO_WALL; //Update the state
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
    V = ADCRead(V_SENSE_ADC); //ADC1
    VBAT = ADCRead(VBAT_SENSE_ADC); //ADC2
    if(V >= VBAT && (switchingState !=  CONNECTED_TO_BAT)){
      switchBatToWall();
    }
    else if(V < VBAT && (switchingState != CONNECTED_TO_WALL)){
      switchWallToBat();
    }
  }
}


//Triggers when ESTOP value changes
ISR(PCINT2_vect) {
    if(!digitalRead(ESTOP_SW)){ //Button 0 when pressed, RED LED ON, Power for motors is not allowed
      EStopPressed=1;
      bitWrite(PORTC, PIN_MOTOR_PWR_CTRL, 0);
      bitWrite(PORTB, ESTOP_LED_R, 0);
      bitWrite(PORTB, ESTOP_LED_G, 1);
    }
    else{
      EStopPressed=0;
      if(switchingState == CONNECTED_TO_BAT){ //GREEN LED ON, power for is allowed when connected to bat
        bitWrite(PORTC, PIN_MOTOR_PWR_CTRL, 1);
        bitWrite(PORTB, ESTOP_LED_R, 1);
        bitWrite(PORTB, ESTOP_LED_G, 0);
      }
    }
}


void sendDataOverI2C(){

  //Read currents and send them to STM on request
  //No need to read Voltage again because interrupt updates them
  uint16_t MtrCurrent = ADCRead(I_MOTORS_SENSE_ADC);
  uint16_t NucCurrent = ADCRead(I_NUC_SENSE_ADC);
  
  // Convert the numbers to bytes
  byte byteArr[8];
  byteArr[0] = highByte(MtrCurrent);
  byteArr[1] = lowByte(MtrCurrent);
  
  byteArr[2] = highByte(NucCurrent);
  byteArr[3] = lowByte(NucCurrent);

  byteArr[4] = highByte(V);
  byteArr[5] = lowByte(V);

  byteArr[6] = highByte(VBAT);
  byteArr[7] = lowByte(VBAT);

  Wire.beginTransmission(0x12); //STM'S address
  Wire.write(byteArr, sizeof(byteArr)); //Send byte array to STM
  Wire.endTransmission();
  
}
