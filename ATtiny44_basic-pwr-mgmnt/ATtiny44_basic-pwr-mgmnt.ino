#define POWER_ON PA0 
#define MOTOR_ON PA1
#define ESTOP_SW PA2 // normally closed i.e if button is pressed, signal goes high
#define VBAT_SENSE PA3 // signal for STM
#define USB_SENSE PA4 // 5 V from NUC 
#define ESTOP_LED_R PA5 // common anode i.e pull down to turn on
#define ESTOP_LED_G PA6 // common anode i.e pull down to turn on
#define BUZZER PA7

#define POWEROFF_REQ PB0 // external poweroff request
#define POWEROFF PB1 // power off signal to other devices 
#define POWER_SW PB2 // pulled high i.e if button is pressed, signal goes low

#define POWER_ON_TIME_MS 3000
#define SOFT_SHUTDOWN_TIME_MS 3000
#define HARD_SHUTDOWN_TIME_MS 10000
#define SHUTDOWN_TIMEOUT_MS 30000

#define VBAT_SENSE_

void setup()
{
  // 1 for output, 0 for input
  DDRA = (1 << POWER_ON) | (1 << MOTOR_ON) | (0 << ESTOP_SW) | (0 << VBAT_SENSE) | (0 << USB_SENSE) | (1 << ESTOP_LED_R) | (1 << ESTOP_LED_G) | (1 << BUZZER);
  DDRB = (0 << POWEROFF_REQ) | (1 << POWEROFF) | (0 << POWER_SW);

  PORTA = (1 << ESTOP_SW); // ESTOP pullup

  digitalWrite(ESTOP_LED_R, LOW);
  digitalWrite(ESTOP_LED_G, LOW);
}

byte prevButtonState = 1;
byte prevEstopState = 0;
byte systemOn = 0;
byte buttonState;
byte estopState;
byte usbSenseState;
byte vbatSenseState;

uint32_t currentTime;
uint32_t timer;
uint32_t vbatTimer=0;

void loop()
{
  currentTime = millis();
  buttonState = PINB & (1 << POWER_SW);

  // if button is not pressed, timer will be updated
  if (buttonState) timer = currentTime;
  else (!buttonState) ;
  
  // power on sequence
  if (!systemOn)
  {
    if (currentTime - timer > POWER_ON_TIME_MS)
    {
      digitalWrite(POWER_ON, HIGH);
      estopState = PINA & (1 << ESTOP_SW);
      
      if (estopState)
      {
        digitalWrite(MOTOR_ON, LOW);
        digitalWrite(ESTOP_LED_G, HIGH);
        digitalWrite(ESTOP_LED_R, LOW);
      }
      
      else 
      {
        digitalWrite(MOTOR_ON, HIGH);
        digitalWrite(ESTOP_LED_G, LOW);
        digitalWrite(ESTOP_LED_R, HIGH);
      }
      timer = currentTime; // reset timer

      systemOn = 1;

      beep(1000);
      beep(800);
      beep(600);
    }
  }

  else
  {
    estopState = PINA & (1 << ESTOP_SW);
    if (estopState && !prevEstopState) // ESTOP pressed
    {
      beep(1200);
      digitalWrite(MOTOR_ON, LOW);
      digitalWrite(ESTOP_LED_R, LOW);
      digitalWrite(ESTOP_LED_G, HIGH);
    }

    else if (!estopState && prevEstopState) // ESTOP released
    {
      beep(300);
      digitalWrite(MOTOR_ON, HIGH);
      digitalWrite(ESTOP_LED_R, HIGH);
      digitalWrite(ESTOP_LED_G, LOW);
    }

    // shutdown sequence
    if (currentTime - timer > SOFT_SHUTDOWN_TIME_MS)
    {
      digitalWrite(POWEROFF, HIGH);
      digitalWrite(MOTOR_ON, LOW);
      digitalWrite(ESTOP_LED_G, LOW);
      digitalWrite(ESTOP_LED_R, LOW);
      
      beep(600);
      beep(800);

      // usbSenseState = PINA & (1 << USB_SENSE);
      int usbsenseTEMP = analogRead(USB_SENSE);
      // while (usbSenseState)
      while (usbsenseTEMP > 200)
      {
        currentTime = millis();
        if ((currentTime - timer > HARD_SHUTDOWN_TIME_MS) && !buttonState)
        {
          break;
        }
        if (currentTime - timer > SHUTDOWN_TIMEOUT_MS) 
        {
          break;
        }
        // ESTOP led blink yellow
        delay(500);
        PINA = (1 << ESTOP_LED_G) | (1 << ESTOP_LED_R); 

        buttonState = PINB & (1 << POWER_SW);
        usbsenseTEMP = analogRead(USB_SENSE);
        // usbSenseState = PINA & (1 << USB_SENSE);
      }

      beep(1000);
      beep(1200);

      digitalWrite(POWER_ON, LOW);
      digitalWrite(ESTOP_LED_G, HIGH);
      digitalWrite(ESTOP_LED_R, HIGH);

      systemOn = 0;
      timer = currentTime; // reset timer
    }
    prevEstopState = estopState;
  }
  prevButtonState = buttonState;

  checkVoltage();
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

// Limit calculation:
// VBAT=13 V through R1=100k and R2=15k voltage divider forms 1.7 V,
// which for 10bit ADC is CODE=1024/3.3*VBAT*R2/(R1+R2) or 1024/3.3*13*15/115 = 526
void checkVoltage()
{
  if (analogRead(VBAT_SENSE) <= 526) 
  {
    if (currentTime - vbatTimer > 3000)
    {
      for (uint8_t i = 0; i < 3; i++){
        for (uint16_t j = 0; j < 100; j++)
        {
          digitalWrite(BUZZER, HIGH);
          delayMicroseconds(300);
          digitalWrite(BUZZER, LOW);
          delayMicroseconds(300);  
          
        }
        delay(100);
      }
      vbatTimer = currentTime; // reset timer for a while
    }
  }
  else
  {
    vbatTimer = currentTime; // vbat higher than threshold, reset timer
  }
}
