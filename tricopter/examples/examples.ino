#include "Arduino.h"

template <typename T>
inline T const& max_(T const& a, T const& b)
{
  return a < b ? b : a;
}
template float  max(float, float);  
void setup() {
  // put your setup code here, to run once:
  Serial.begin(57600);

}

void loop() {
  // put your main code here, to run repeatedly:
  
  Serial.println("Hello Worlds");
  max(23,24);
  delay(1000);
}
