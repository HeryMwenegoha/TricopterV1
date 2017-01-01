#include <mavlink.h>
#include <AP_Parameters.h>
#include <AP_Airspeed.h>
#include <AP_Baro.h>
#include <AP_Compass.h>
#include <AP_GPS.h>  
#include <AP_AHRS.h>
#include <AC_PID.h>
#include <AC_P.h>
#include <AC_THRPID.h>
#include <AC_ACCELPID.h>
#include <AP_Radio.h>
#include <AP_Sensors.h>
#include <Servo.h>
#include <AP_HgtFilter.h>
#include <AP_HgtFilter_AQ.h>
#include <AP_BatteryVolts.h>
#include "AP_Define.h"
#include "AP_WayPoint.h"
//#include "AP_Program.h"
#include "AP_Mavlink.h"
#if PLANE_TYPE == 0
#error Change Servo Refresh rate to 125Hz
#endif

#define LEDPIN 12

#define PRINT(x) Serial.print(x)

AP_Radio        AP_radio;
AP_Airspeed     AP_airspeed(&(AP_params.ParameterStorage.list.arspdEnabled));
AP_Baro         AP_baro;
AP_GPS          AP_gps;
AP_Sensors      AP_sensors; // carries IMU and Compass
AP_AHRS         AP_ahrs(&AP_airspeed, AP_gps, &AP_ins, &AP_compass, AP_baro); // GPS is passed by reference
AP_WayPoint     AP_waypoint;
AP_HgtFilter_AQ AP_hgtfilter_aq(AP_ahrs, &AP_params);
AP_HgtFilter    AP_hgtfilter(AP_ahrs, AP_params);
AP_Program      AP_program(&AP_params, &AP_ahrs, &AP_gps, &AP_waypoint, &AP_radio);
AP_BatteryVolts AP_batteryvolts;
AP_Mavlink      AP_mavlink(AP_params , AP_ahrs , AP_gps,AP_ins,AP_compass,AP_airspeed, AP_baro, AP_waypoint, AP_program, AP_hgtfilter_aq, AP_hgtfilter, AP_batteryvolts);

uint64_t last_stream_usec;
bool led_state;
uint32_t blink_msec;
void setup()
{
  // Using only one serial port
  // SD library uses Pin4
  Serial.begin(57600);
  pinMode(LEDPIN, OUTPUT);
  pinMode(A0, INPUT);

  AP_params.initialise(&Serial); 
  AP_waypoint.initialise();
  AP_mavlink.initialise(&Serial);
  AP_gps.initialise(&Serial1);
  AP_sensors.initialise(); 
  AP_baro.initialise();
  AP_program.initialise();
  AP_batteryvolts.initialise();

  // Timer cant be used for SD card management.
  //Timer1_setup();

  mainloop_lastTime  = millis() - mainloop_rate;
  last_stream_usec   = micros();
}

byte medium_loop_counter = 0;
void loop()
{
  precise_timing_loop();
  uint32_t mainloop_currentTime = millis();
  if(mainloop_currentTime - mainloop_lastTime >= mainloop_rate){
    float Gdt         = mainloop_currentTime - mainloop_lastTime;
    mainloop_lastTime = mainloop_currentTime;    
    if(Gdt > 50){
      Serial.print(F("Loop Speed Error: ")); 
      Serial.println(Gdt);
    }  
    fast_loop();
    medium_loop();
    blink_led();
  }
}


void precise_timing_loop()
{  
  // Read incoming GPS stream
  AP_gps.read();

  // Accumulate sensor readings
  AP_sensors.accumulate();

  uint64_t now = micros();
  float DT     = (now - last_stream_usec) * 1e-3f;
  if(DT       >= 10){
    last_stream_usec  = now;
    
    // Get backend object and update front end objects
    AP_sensors.update();
    
    // Process Front end compass values from raw mag values into proper mag fields
    AP_compass.update();
    
    // Process Front end imu raw values into proper scaled imu values (rotation rate and accelerations)
    AP_ins.update();
    
    // Perform GPS based direct cosine matrix calculations with scaled compass and imu readings to get euler angles
    AP_ahrs.update();
    
    // Autopilot Programme   
    AP_program.update(_servo_out);
  }

  AP_hgtfilter_aq.update();

    // Read incoming stream from Serial0
  AP_mavlink.readstream();
}


/*
 * 50Hz Loop
 */
void fast_loop()
{ 
  // Send mavlink stream on Serial0
  AP_mavlink.sendstream();

  // climbrate filter
  AP_hgtfilter.update();

  // Read BMP180
  AP_baro.read();
}


/* 
 * 10Hz Loop
 */
void medium_loop(){
  switch(medium_loop_counter)
  {
    case 0:
      medium_loop_counter++;
      AP_batteryvolts.read();
      break;

    case 1:
      medium_loop_counter++;
      #if AQ_FILTER
      Serial.print(millis());
      Serial.print("\t");
      Serial.print(AP_baro.get_altitude());
      Serial.print("\t");
      Serial.print(AP_hgtfilter.altitude());
      Serial.print("\t");
      Serial.print(AP_hgtfilter.climbrate()*100); 
      Serial.print("\t");
      Serial.print(AP_hgtfilter_aq.altitude());
      Serial.print("\t");
      Serial.print(AP_hgtfilter_aq.climbrate()*100);
      Serial.print("\t");
      Serial.print(AP_ahrs.acc.x);
      Serial.print("\t");
      Serial.print(AP_ahrs.acc.y);
      Serial.print("\t");
      Serial.println(AP_ahrs.acc.z);
      #endif
      break;

    case 2:
      medium_loop_counter++;
      break;

    case 3:
      medium_loop_counter++;
      //Serial.println(-((AP_ahrs.dcm() * vector3f(AP_ahrs.acc.x, AP_ahrs.acc.y, AP_ahrs.acc.z)).z + 9.81));

      break;

    case 4:
      medium_loop_counter = 0;
      AP_baro.update();
      break;
  }
}

void blink_led()
{
  led_counter++;

  byte events = 0;

  float battery_voltage = AP_batteryvolts.get_adc() * AP_params.ParameterStorage.list.PowerModule_Gain;

  //Serial.println(battery_voltage);

  // Battery very low event
  if(battery_voltage < 10.75)
  {
    events = 1;
  }
  else
  {
    events = 0;
  }
  
  switch(events){
    case 0:
    blinker(1000);
    break;

    case 1:
    blinker(100);
    break;
  }

}

void blinker (uint16_t time_delay){
  time_delay = time_delay/2;
  uint32_t now = millis();
  if((now - blink_msec) >= time_delay){
     blink_msec  = now;
     led_state   = !led_state;
     digitalWrite(LEDPIN, led_state);
  }  
}


void print_radio(){
  PRINT(AP_radio.chan1());
  PRINT(F("   "));
  PRINT(AP_radio.chan2());
  PRINT(F("   "));
  PRINT(AP_radio.chan3());
  PRINT(F("   "));
  PRINT(AP_radio.chan4());   
  PRINT(F("   "));
  PRINT(AP_radio.chan8()); 
  PRINT('\n');
}

void print_euler(){
  PRINT(ToDeg(AP_ahrs.roll));
  PRINT("  ");
  PRINT(ToDeg(AP_ahrs.pitch));
  PRINT("  ");
  PRINT(ToDeg(AP_ahrs.yaw));
  PRINT("  ");
  PRINT(ToDeg(AP_ahrs.rollrate));
  PRINT("  ");
  PRINT(ToDeg(AP_ahrs.pitchrate));
  PRINT("  "); 
  PRINT(ToDeg(AP_ahrs.yawrate));
  PRINT('\n');  
}

void Timer1_setup(){
  cli();                            // diable all interrupts
  TCCR1A = 0;                       // has to be cleared as it affects normal mode operation
  TCCR1B = 0;
  TCNT1  = 0; 
   
  TCCR1B = (0 << CS12) | (1 <<CS11) | (1 << CS10); // Prescaler 64 giving 3.8Hz
  
  TIFR1  = 1 << TOV1;              // Clear Overflow flag
  TIMSK1 = 1 << TOIE1;             // Enable overflow interrupt
  sei();                            // enable all interrupts
}

ISR(TIMER1_OVF_vect){
  uint8_t status = SREG; 
  cli();      
  // do stuff
  SREG        = status;
}
/*
void call_dx9(){
  AP_DSM *ptr = &AP_dsm;
  (ptr->read_stream)();
}
Timer Thread for Satellite Receiver
Timer8.attachInterrupt(call_dx9);
Timer8.start(5000);
*/
