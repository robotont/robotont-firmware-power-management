#define PIN_DBG_LED_G PD6
#define PIN_DBG_LED_R PD5
#define PIN_POWER_SW PD2
#define PIN_BAT_PWR_CTRL 7 // PortB
#define PIN_WALL_PWR_CTRL PD4
#define PIN_SYS_PWR_CTRL 3 //PortA
#define PIN_MOTOR_PWR_CTRL 7 //PortC


void setup() {
  pinMode(PIN_DBG_LED_G, OUTPUT);
  pinMode(PIN_DBG_LED_R, OUTPUT);
  //pinMode(PIN_BAT_PWR_CTRL, OUTPUT);
  bitSet(DDRB, PIN_BAT_PWR_CTRL); //Set as output
  pinMode(PIN_WALL_PWR_CTRL, OUTPUT);
  bitSet(DDRA, PIN_SYS_PWR_CTRL);
  bitSet(DDRA, PIN_MOTOR_PWR_CTRL);
  pinMode(PIN_POWER_SW, INPUT);
  digitalWrite(PIN_DBG_LED_R, LOW);
  digitalWrite(PIN_DBG_LED_G, LOW);
  //digitalWrite(PIN_BAT_PWR_CTRL, LOW);
  bitClear(PORTB, PIN_BAT_PWR_CTRL);
  digitalWrite(PIN_WALL_PWR_CTRL, LOW);
  bitClear(PORTA, PIN_SYS_PWR_CTRL);
  bitClear(PORTC, PIN_MOTOR_PWR_CTRL);
}

void loop() {
  //digitalWrite(PIN_BAT_PWR_CTRL, HIGH);
  digitalWrite(PIN_DBG_LED_G, HIGH); 
  delay(500);
  //digitalWrite(PIN_BAT_PWR_CTRL, LOW);                       
  digitalWrite(PIN_DBG_LED_G, LOW);
  delay(500);
  if (!digitalRead(PIN_POWER_SW))
  {
    if (!digitalRead(PIN_DBG_LED_R))
    {
      // Turn power from bat to sys and motors on
      digitalWrite(PIN_DBG_LED_R, HIGH);
      bitSet(PORTB, PIN_BAT_PWR_CTRL);
      bitSet(PORTA, PIN_SYS_PWR_CTRL);
      bitSet(PORTC, PIN_MOTOR_PWR_CTRL);
      //digitalWrite(PIN_BAT_PWR_CTRL, HIGH);
      //digitalWrite(PIN_SYS_PWR_CTRL, HIGH);
    }
    else
    {
      // Turn power off
      digitalWrite(PIN_DBG_LED_R, LOW);
      bitClear(PORTB, PIN_BAT_PWR_CTRL);
      bitClear(PORTA, PIN_SYS_PWR_CTRL);
      bitClear(PORTC, PIN_MOTOR_PWR_CTRL);
      //digitalWrite(PIN_BAT_PWR_CTRL, LOW);
      //digitalWrite(PIN_SYS_PWR_CTRL, LOW);
    }
  }
}
