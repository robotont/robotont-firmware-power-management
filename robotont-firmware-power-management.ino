
#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdio.h>
#include <Wire.h>
#include <Arduino.h>

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

#define F_CPU 8000000UL

enum PowerState{
  POWER_OFF,
  POWER_ON
};

enum SwitchingState{
  INIT,
  CONNECTED_TO_WALL,
  CONNECTED_TO_BAT
};

enum ADCChannelState{
  Vchannel,
  VBATchannel,
  NUCchannel,
  MTRchannel
};

enum PowerState powerState = POWER_OFF;
enum SwitchingState switchingState = INIT;
enum ADCChannelState ADCchannel = Vchannel;

uint8_t ADC_ISR_Counter = 0;
uint8_t EStopPressed = 0;
uint8_t LastPwrButton = 1;

uint16_t V;
uint16_t VBAT;
uint16_t MtrCurrent;
uint16_t NucCurrent;

//Loop timer variables
unsigned long ledInterval=500; //Interval for led blinking (milliseconds)
unsigned long i2cInterval=100; //Interval for i2c communication (milliseconds)
unsigned long powerOnTime=600; //How long you need to press the power button to turn on robot (milliseconds)
unsigned long powerOffTime=1200; //How long you need to press the power button to turn off robot (milliseconds)

unsigned long startTimeforLed;
unsigned long startTimeforI2C;
unsigned long startTimeforPwrButton;
unsigned long prevTimeforLed=0;
unsigned long prevTimeforI2c=0;
unsigned long pwrButtonTimer=millis();
uint8_t ledState=0;

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
  
  const int interruptFrequency = 3000UL;
  
  TCCR1A &= 0;
  TCCR1B |= (0 << CS12) | (1 << CS11) | (0 << CS10) | (1 << WGM12); //CTC and 8 pre
  TCNT1 = 0; //Counter to zero
  OCR1A = F_CPU / (2 * 8 *interruptFrequency) - 1;
  TIMSK1 |= (1 << OCIE1A); //Enable interrupt

  
  ADCSRA |= (1 << ADEN) | (1 << ADIE) | (0 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); //Prescaler 8
  ADCSRB &= (0 << ADTS2) | (0 << ADTS1) | (0 << ADTS0);
  ADMUX |= (1 << REFS0);
  
  ADCSRA |= (1 << ADSC); // Start conversion
  
  Wire.begin(); // join i2c bus as master
  
  sei(); //Enable interrupts

}


void loop() {

  startTimeforLed = millis();

  if(startTimeforLed-prevTimeforLed > ledInterval){
    prevTimeforLed=startTimeforLed;
    bitWrite(PORTD, PIN_DBG_LED_G, ledState);
    ledState=!ledState;
    if(VBAT<680 && switchingState==CONNECTED_TO_BAT){
      beep(1000);
    }
  }

  startTimeforI2C = millis();
  if(startTimeforI2C-prevTimeforI2c > i2cInterval){
    prevTimeforI2c = startTimeforI2C;
    sendDataOverI2C();
  }

  
  uint8_t PwrButton = digitalRead(POWER_SW);
  if(PwrButton){
    pwrButtonTimer = millis();
  }
  startTimeforPwrButton = millis();
  if (startTimeforPwrButton - pwrButtonTimer > powerOnTime && powerState == POWER_OFF){
    bitWrite(PORTD, PIN_DBG_LED_R, 1);
    powerState = POWER_ON;
    // Buzzer output that system is ON
    beep(1000);
    beep(800);
    beep(600);
    
    if(EStopPressed == 1){
      bitWrite(PORTB, ESTOP_LED_R, 0);
      bitWrite(PORTB, ESTOP_LED_G, 1);
    }
    else{
      bitWrite(PORTB, ESTOP_LED_R, 1);
      bitWrite(PORTB, ESTOP_LED_G, 0);
    }
    
    pwrButtonTimer = millis();
  }
  else if (startTimeforPwrButton - pwrButtonTimer > powerOffTime && powerState == POWER_ON){
    // Turn power off
    powerState = POWER_OFF;
    switchingState = INIT;
    bitWrite(PORTD, PIN_DBG_LED_R, 0);
    bitWrite(PORTB, PIN_BAT_PWR_CTRL, 0);
    bitWrite(PORTD, PIN_WALL_PWR_CTRL, 0);
    bitWrite(PORTC, PIN_MOTOR_PWR_CTRL, 0);
    bitWrite(PORTA, PIN_SYS_PWR_CTRL, 0);
    //ESTOP LEDS OFF TO SAVE BAT
    bitWrite(PORTB, ESTOP_LED_R, 1);
    bitWrite(PORTB, ESTOP_LED_G, 1);
    //Buzzer output, system is OFF
    beep(600);
    beep(800);
    beep(1000);
    pwrButtonTimer = millis();
  }
  
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


ISR(ADC_vect){

  ADC_ISR_Counter+=1;
  uint8_t low = ADCL;
  uint8_t high = ADCH;
  uint16_t ADC_value = (high << 8) | low; 

  if(ADCchannel == VBATchannel){ //VBAT TO V
    VBAT = ADC_value;
    ADMUX = (ADMUX & 0xF0) | (V_SENSE_ADC & 0x0F);
    ADCchannel = Vchannel;
  }
  else if (ADCchannel == Vchannel){ //V TO VBAT
    V = ADC_value;
    if(ADC_ISR_Counter < 200){ 
      ADMUX = (ADMUX & 0xF0) | (VBAT_SENSE_ADC & 0x0F);
      ADCchannel = VBATchannel;
    }
    else{
      ADMUX = (ADMUX & 0xF0) | (I_NUC_SENSE_ADC & 0x0F);
      ADCchannel = NUCchannel;
    }
  }
  else if(ADCchannel == NUCchannel){
    NucCurrent = ADC_value;
    ADMUX = (ADMUX & 0xF0) | (I_MOTORS_SENSE_ADC & 0x0F);
    ADCchannel = MTRchannel;
  }
  else if(ADCchannel == MTRchannel){
    MtrCurrent = ADC_value;
    ADMUX = (ADMUX & 0xF0) | (VBAT_SENSE_ADC & 0x0F);
    ADCchannel = VBATchannel;
    ADC_ISR_Counter=0;
  }
                
  ADCSRA |= (1 << ADSC);
                                              
}

ISR(TIMER1_COMPA_vect) {
  
  if(powerState == POWER_ON){
    //We dont need VBAT here right now
    //Added 680 for reference value as placeholder
    if(V >= 680 && (switchingState !=  CONNECTED_TO_BAT)){
      switchBatToWall();
    }
    else if(V < 680 && (switchingState != CONNECTED_TO_WALL)){
      switchWallToBat();
    }
  }
}


//Triggers when ESTOP value changes
ISR(PCINT2_vect) {
    EStopPressed = !digitalRead(ESTOP_SW); //Button 0 when pressed, 
    if(powerState == POWER_ON){
        if(EStopPressed == 1){ //RED LED ON, Power for motors is not allowed
          bitWrite(PORTC, PIN_MOTOR_PWR_CTRL, 0);
          bitWrite(PORTB, ESTOP_LED_R, 0);
          bitWrite(PORTB, ESTOP_LED_G, 1);
        }
        else{ //ESTOP NOT PRESSED
          if(switchingState == CONNECTED_TO_BAT){ //GREEN LED ON, power motors for is allowed when connected to bat
            bitWrite(PORTC, PIN_MOTOR_PWR_CTRL, 1);
            bitWrite(PORTB, ESTOP_LED_R, 1);
            bitWrite(PORTB, ESTOP_LED_G, 0);
          }
        }
    }else{ //if sys is off, the leds are off
      bitWrite(PORTB, ESTOP_LED_R, 1);
      bitWrite(PORTB, ESTOP_LED_G, 1);
    }
}


void sendDataOverI2C(){ 
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
