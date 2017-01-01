typedef struct message_{
  float a;
  float b;
  float c;
  float d;
  float e;
}_msg;

_msg msg;

_msg _msg_[3] = {
  {},
  {},
  {}  
};

_msg msg_[25];

void setup() {
  // put your setup code here, to run once:

  Serial.begin(57600);
}

float buffer[80];

void pass_byvalue(){
  
}

void loop() {
  // put your main code here, to run repeatedly:
  //_msg msg_a.a = 20;
  
  msg.a = 20;
  buffer[0] = 78;

  msg_[0].a = 20;
}
