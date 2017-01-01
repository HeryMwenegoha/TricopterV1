/*
Function  pins per Ch. A  pins per Ch. B
Direction D12 D13
PWM D3  D11
Brake D9  D8
Current Sensing A0  A1
*/
#define DIRA  12
#define BRAKEA  9
#define PWMA  3

void setup() {
  // put your setup code here, to run once:
  Serial.begin(57600);

  delay(2000);
  Serial.println("Start");

  pinMode(DIRA, OUTPUT);
  pinMode(BRAKEA, OUTPUT);
  pinMode(PWMA, OUTPUT);

  Serial.println("Forward");
  digitalWrite(DIRA, HIGH);
  for(int i = 0; i < 10; i++){
    analogWrite(PWMA, i*20);
    Serial.println(i*20);
    delay(1000);
  }
  digitalWrite(BRAKEA, HIGH);

  delay(1000);

  Serial.println("Backward");
  digitalWrite(BRAKEA, LOW);
  digitalWrite(DIRA, LOW);
  for(int i = 0; i < 10; i++){
    analogWrite(PWMA, i*20);
    Serial.println(i*20);
    delay(1000);
  }
  digitalWrite(BRAKEA, HIGH);

  Serial.println("DOne");
}

void loop() {
  // put your main code here, to run repeatedly:

}
