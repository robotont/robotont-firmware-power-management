#define POWER_ON PA0 
#define MOTOR_ON PA1
#define ESTOP_SW PA2 // normally closed i.e if button is pressed, signal goes low
#define ESTOP_SIG PA3 // signal for STM
#define USB_SENSE PA4 // 5 V from NUC 
#define ESTOP_LED_R PA5 // common anode i.e pull down to turn on
#define ESTOP_LED_G PA6 // common anode i.e pull down to turn on
#define BUZZER PA7

#define POWEROFF_REQ PB0 // TODO 
#define POWEROFF PB1 // TODO 
#define POWER_SW PB2 // normally closed i.e if button is pressed, signal goes low

#define CNT_MULT 1/16.0

void setup()
{
  // 1 for output, 0 for input
  DDRA = (1 << POWER_ON) | (1 << MOTOR_ON) | (1 << BUZZER) | (0 << ESTOP_SW) | (1 << ESTOP_SIG);
  DDRB = 0 << POWER_SW;

  digitalWrite(ESTOP_LED_R, LOW);
  digitalWrite(ESTOP_LED_G, LOW);
}

byte prevButtonState = 0;
unsigned long counter = 0;
byte systemOn = 0;

void loop()
{
  byte buttonState = PINB & (1 << POWER_SW);
  byte estopState = PINA & (1 << ESTOP_SW);

  if (!estopState) // ESTOP pressed
  {
    digitalWrite(MOTOR_ON, LOW);
    digitalWrite(ESTOP_LED_R, LOW);
    digitalWrite(ESTOP_LED_G, HIGH);
  }

  // counter counts up only if button held down
  if (!buttonState ) counter++;
  else counter = 0;

  // power on sequence
  if (!systemOn)
  {
    if (counter > 300000)
    {
      digitalWrite(POWER_ON, HIGH);
      digitalWrite(MOTOR_ON, LOW);
      counter = 110000; // hack to not turn on motors instantly
      systemOn = 1;

      beep(1000);
      beep(800);
      beep(600);
    }
  }

  else
  {
    if (!prevButtonState && buttonState) // button released
    {
      if (counter > 5000 && counter < 10000) // button press was brief
      {
        if (!(PORTA & (1 << MOTOR_ON))) // turn motors on
        {
          digitalWrite(MOTOR_ON, HIGH);
          digitalWrite(ESTOP_LED_G, LOW);
          digitalWrite(ESTOP_LED_R, HIGH);
        }
        else
        {
          digitalWrite(MOTOR_ON, LOW);
          digitalWrite(ESTOP_LED_G, LOW);
          digitalWrite(ESTOP_LED_R, LOW);
        }
      }
    }

    // shutdown sequence
    if (counter > 400000)
    {
      digitalWrite(MOTOR_ON, LOW);
      digitalWrite(POWER_ON, LOW);
      
      // PORTA = (0 << POWER_ON) | (0 << MOTOR_ON);  
      counter = 0;
      systemOn = 0;

      beep(600);
      beep(800);
      beep(1000);
    }
  }
  prevButtonState = buttonState;
} 

// arbitrary note pitch (period in us)
void beep(uint16_t note)
{
  for (uint8_t i = 0; i < 100; i++)
  {
    digitalWrite(BUZZER, HIGH);
    delayMicroseconds(note);
    digitalWrite(BUZZER, LOW);
    delayMicroseconds(note);
  }
}