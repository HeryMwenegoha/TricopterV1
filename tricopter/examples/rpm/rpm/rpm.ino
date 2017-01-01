#include <LiquidCrystal.h>
#define NO_PORTB_PINCHANGES
#include "PinChangeInt.h"

// initialize the library with the numbers of the interface pins
#define PIN 7
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

void setup() {
  Serial.begin(57600);
  
  lcd.begin(16, 2);
  lcd.print("hello, world!");

  pinMode(PIN, INPUT);
  digitalWrite(PIN, LOW);
  attachInterrupt(PIN, read, CHANGE);
}

volatile uint32_t timeHigh = 0;
volatile uint32_t timeLow  = 0;
volatile uint32_t pulse    = 0;
volatile boolean  on       = false;
volatile uint32_t counter  = 0;
void read()
{
 if((digitalRead(PIN) == HIGH ))
  {
    Serial.println("High");
    timeHigh = micros();
  }
  if((digitalRead(PIN) == LOW))
  {
    on = false;
    timeLow = micros();
  }
  
  pulse = timeHigh - timeLow;
  if(abs(pulse) == 800){
    counter++;
  }
}

uint64_t update_usec;
void loop() {
  uint64_t tnow =micros();
  if(tnow - update_usec >= 1000000)
  {
    float dt = (tnow - update_usec) * 1e-6;
    float rps = counter;
    counter   = 0;
    float rpm = rps * 60;
    uint32_t _RPM = static_cast<uint32_t>(rpm);
    
    update_usec = tnow;
   
    // set the cursor to column 0, line 1
    // (note: line 1 is the second row, since counting begins with 0):
    lcd.setCursor(0, 1);
    
    // print the number of seconds since reset:
    lcd.print("Pulse (us): ");
  
    lcd.setCursor(11, 1);
    lcd.print(_RPM);
  }
}
