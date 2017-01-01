#pragma once
#include "AP_Mavlink.h"

/*
 * Notes:
 * Change Pins to Follow the Mega 2560 Convention
 */
 /*
#define  pin_one      34
#define  pin_two      36
#define  pin_three    22
#define  yawpin       40
*/

#define  pin_one      30
#define  pin_two      32
#define  pin_three    34
#define  yawpin       36

#define  gcs1pin      42
#define  gcs2pin      44

class AP_Program
{
   public:
   AP_Program(AP_Storage *,  AP_AHRS *, AP_GPS *, AP_WayPoint *, AP_Radio *);
   void initialise();
   void update(servo_out &ServoChan);
   byte flight_mode;

  float _climbrate;
  float _height;
     
   private:
   AP_Storage  *_parameter; 
   AP_AHRS     *_ahrs;
   AP_GPS      *_gps;
   AP_WayPoint *_waypoint; 
   AP_Radio    *_radio;

   vector3f     _demanded_accel_body;

   uint64_t     update_stab_usec;
   float throttle;
   float rollchan;
   float pitchchan;
   float yaw_dem;
   uint16_t  min_rec;
   uint16_t  max_rec;

   bool  armed;
   bool  _hdg_hold;
   bool  _alt_hold;
   float _altitude_to_hold;

   float climb_sum;
   byte  count_;
  
   float thr_integrator;
   float _derivative;
   float _last_demanded_rate;
    
   uint32_t _last_althold_msec;
   uint32_t arm_msec;
   uint32_t disarm_msec;
   uint32_t _last_msec;
   uint64_t _throttle_50Hz_usec;
   
   AC_PID       rate_pid_roll;
   AC_PID       rate_pid_pitch;
   AC_PID       rate_pid_yaw;
   AC_P         stab_pid_roll;
   AC_P         stab_pid_pitch;
   AC_P         stab_pid_yaw;
   AC_THRPID    rate_pid_throttle;
   AC_ACCELPID  rate_pid_accel;

   struct _moving_average{
    public:
    _filter(){
      count    = 0;
      input[0] = input[1] = input[2] = input[3] = input[4] = 0;
    }
    
    initialise(float val){
      input[0] = input[1] = input[2] = input[3] = input[4] = val;
    }
    
    float avg(float val){

      switch(count)
      {
       case 0: 
       input[0] = val;
       break;

       case 1:
       input[1] = input[0];
       input[0] = val;
       break;

       case 2:
       input[2] = input[1];
       input[1] = input[0];
       input[0] = val;
       break;
      }
      count++;
      return (input[0] + input[1] + input[2])/3;
      if(count == 3){
        count  = 0;
      }
    }
    
    private:
    float input[5];
    uint8_t count;
   };
   _moving_average  moving_average;

   float elev;
   float vy;
   float accel_correc_P;
   float accel_correc_I;

   float output;
   Servo motor_one;
   Servo motor_two;
   Servo motor_three;
   Servo servo_yaw;  
   
   void  manual_control(servo_out &ServoChan);
   void  stabilised_control(servo_out &ServoChan);
   void  AltHold(servo_out &motor);


   float pitch_stabilise();
   float roll_stabilise();
   float yaw_stabilise();
   float throttle_rate();
   float throttle_control();
};

AP_Program::AP_Program(AP_Storage *ap_params, AP_AHRS *ap_dcm, AP_GPS * ap_gps, AP_WayPoint *ap_wp, AP_Radio *ap_radio):
rate_pid_roll  (ap_params, ROLLPID) ,
rate_pid_pitch (ap_params, PITCHPID),
rate_pid_yaw   (ap_params, YAWPID)  ,
stab_pid_roll  (ap_params, ROLLPID) ,
stab_pid_pitch (ap_params, PITCHPID),
stab_pid_yaw   (ap_params, YAWPID)  
{
  flight_mode = PREFLIGHT; 
  _parameter  = ap_params;
  _ahrs       = ap_dcm;
  _gps        = ap_gps;
  _waypoint   = ap_wp;
  _radio      = ap_radio;

  armed               = false;
  disarm_msec         = 0;
  arm_msec            = 0;
  _last_althold_msec  = 0;
  yaw_dem             = 0;
  _throttle_50Hz_usec = 0;

  _hdg_hold   = false;
  _alt_hold   = false;

  max_rec   = 1500;
  min_rec   = 1500;
  rollchan  = 1500;
  pitchchan = 1500;

  _demanded_accel_body.zero();
}


void
AP_Program::initialise(){
  update_stab_usec = micros();
  throttle         = 1000;
  
  motor_one.attach(pin_one,1000,2000);
  motor_two.attach(pin_two,1000,2000);
  motor_three.attach(pin_three, 1000,2000);
  servo_yaw.attach(yawpin,      1000, 2000);

  max_rec = _parameter->ParameterStorage.list.max_thr_aux;
  min_rec = _parameter->ParameterStorage.list.min_thr_aux;

  count_    = 0;
  climb_sum = 0;
}

void 
AP_Program::update(servo_out &ServoChan)
{
  uint16_t mode         = _radio->chan8(); //channels[7]; // channel number 8  ID 7
  ServoChan.chan8       = _radio->chan8();
  if(FLIGHT_MODE(mode) != IGNORE)
    flight_mode = FLIGHT_MODE(mode);
  
  _ahrs->set_flightmode(flight_mode);

  uint32_t tnow = millis();
  if(tnow - _last_msec >= 25){
    float DT   = (tnow - _last_msec) * 1e-3f;
    _last_msec = tnow;

    if(_ahrs->healthy())
    {
      static uint8_t inited    = 0   ;
      static float AltErrorI   = 0.0f;
      static float AccScale    = 0.0f;
      static float EstVelocity = 0.0f;
      static float EstAlt      = 0.0f;
      float Kp1                = 0.45f ;               // PI observer velocity gain 
      float Kp2                = 1.0f;                 // PI observer position gain
      float Ki                 = 0.001f;               // PI observer integral gain (bias cancellation)
     
      float AltError = 0.0f;
      float InstAcc  = 0.0f;
      float Delta    = 0.0f;
    
      // Soft start
      if (!inited) {
        inited       = 1;
        EstAlt       = _ahrs->baro().get_altitude();
        EstVelocity  = 0.0f;
        AltErrorI    = 0.0f;
      }
        
      // Estimation Error
      AltError     = _ahrs->baro().get_altitude() - EstAlt; 
      AltErrorI   += AltError;
      
      // Gravity vector correction and projection to the local Z
      InstAcc      = -((_ahrs->dcm() * vector3f(_ahrs->acc.x, _ahrs->acc.y, _ahrs->acc.z)).z + 9.81) + (Ki) * AltErrorI;
      
      // Integrators
      Delta        = InstAcc * DT + (Kp1 * DT) * AltError;
      EstAlt      += ((EstVelocity + Delta * 0.5f) * DT + (Kp2 * DT) * AltError);
      EstVelocity += Delta;

      _height      = 0.2 * EstAlt + 0.8 * _height;
      _climbrate   = 0.2 * EstVelocity + 0.8 * _climbrate;

      #if PRINT
      Serial.print(millis());
      Serial.print("\t");
      Serial.print(_height);
      Serial.print("\t");
      Serial.print(_climbrate * 100);
      Serial.print("\t");
      Serial.print(_ahrs->acc.x);
      Serial.print("\t");
      Serial.print(_ahrs->acc.y);
      Serial.print("\t");
      Serial.println(_ahrs->acc.z);
      #endif
    }
  
  
    #if PRINT_HGT_FILTER
    Serial.print(_climbrate*100);
    Serial.print("\t");
    Serial.println(_height);
    Serial.print("\t");
    Serial.print(_ahrs->altitude_estimate());
    Serial.print("\t");
    Serial.println(-((_ahrs->dcm() * vector3f(_ahrs->acc.x, _ahrs->acc.y, _ahrs->acc.z)).z + 9.81));
    #endif
  }

  switch(flight_mode)
  {
    case MANUAL:
    manual_control(ServoChan);
    break;
    
    case STABILISE:
    stabilised_control(ServoChan);
    break;

    case ALTHOLD:
    AltHold(ServoChan);
    break;
  }

  #if AHRS
  Serial.print("ahrs ");
  Serial.print(ToDeg(_ahrs->roll));
  Serial.print(" ");
  Serial.print(ToDeg(_ahrs->pitch));  
  Serial.print(" ");
  Serial.println(ToDeg(_ahrs->yaw));  
  #endif

  #if AHRS
  Serial.print(millis());
  Serial.print("\t");
  Serial.print(_ahrs->acc.x);
  Serial.print("\t");
  Serial.print(_ahrs->acc.y);  
  Serial.print("\t");
  Serial.print(_ahrs->acc.z);  
  Serial.print("\t");
  Serial.print(_ahrs->baro().get_altitude());  
  Serial.print("\t");
  Serial.print(_height);  
  Serial.print("\t");
  Serial.println(_climbrate);  
  #endif
  
  #if FREEZE
  Serial.print("Chan ");
  Serial.print(_radio->chan1());
  Serial.print(" ");
  Serial.print(_radio->chan2());
  Serial.print(" ");
  Serial.print(_radio->chan3());
  Serial.print(" ");
  Serial.print(_radio->chan4());  
  Serial.print(" ");
  Serial.println(_radio->chan8());
  #endif
}

void 
AP_Program::manual_control(servo_out &ServoChan)
{
  /* 
   *  Motor Arming
   */

  uint16_t arm_sig = _radio->chan3();
  ServoChan.chan1  = _radio->chan1();
  ServoChan.chan2  = _radio->chan2();
  ServoChan.chan3  = _radio->chan3();  
  ServoChan.chan4  = _radio->chan4();  

  /*
   * Establish Min and Max
   */
   max_rec = _parameter->ParameterStorage.list.max_thr_aux;
   min_rec = _parameter->ParameterStorage.list.min_thr_aux;
  
  /* 
   *  write servos
   */

  /*
  Serial.print(_radio->chan1());
  Serial.print(_radio->chan2());
  Serial.print(_radio->chan3());
  Serial.println(_radio->chan4());
  */
  
  motor_one.writeMicroseconds(arm_sig);
  motor_two.writeMicroseconds(arm_sig);
  motor_three.writeMicroseconds(arm_sig);
  servo_yaw.writeMicroseconds(ServoChan.chan4); 
}

void 
AP_Program::stabilised_control(servo_out &motor)
{ 
  //Serial.println("Stabilise");
  /*
   * Initial reset
   */
   uint64_t now     = micros();
   float    DT      = (now  - update_stab_usec) * 1e-6f;
   update_stab_usec = now;
   if(DT > 1.0f){
    throttle        = _radio->chan3();
    DT              = 0.01f;
    arm_msec        = millis();
    disarm_msec     = millis();
    yaw_dem         = ToDeg(_ahrs->yaw);
    _hdg_hold       = false;
   } 

  // arming works well  
  if(_radio->chan4() > 1900     && _radio->chan3() < 1100){
    arm_msec   = millis();  
    if((arm_msec - disarm_msec) >= 2000){
      if(_ahrs->healthy()){
        armed  = true; // only arm with healthy IMU if not healthy then do not arm
      }
      disarm_msec = arm_msec;
      // send message armed
    }
  }
  else if(_radio->chan3() < 1100){
    // if disarmed update both milliseconds
    disarm_msec = millis();
    if(armed  == false){
     arm_msec  = millis(); 
    }    
    if((disarm_msec - arm_msec) >= 3000){
      armed     = false;
      arm_msec  = disarm_msec;
      // send message disarmed
    }
  }

  /*
   * Throttle Functions
   * SlewRate is the percentage throttle change over the full scale range in one second
   * SR = (ΔThrottle/FullScaleRange)/dt
   */
  throttle = _radio->chan3(); 
  throttle = constrain_float(throttle, min_rec, max_rec - 50);
  float range_lim = max_rec - min_rec;
  float range = throttle - min_rec;
  float perc  = (range/range_lim) * 100;      
  float max_range = (max_rec  - min_rec);
  float mid_point = 0.5f * (max_rec + min_rec);
  float limit     =  max_range*0.5f - fabs(throttle - mid_point);
 
  /* 
   * populate servo channel  
   */
  float roll_sig  = roll_stabilise();
  float pitch_sig = pitch_stabilise();
  float yaw_sig   = yaw_stabilise();
  
  if(perc <= 10){
    rate_pid_roll.reset();
    rate_pid_pitch.reset();
    rate_pid_yaw.reset(); 
    roll_sig  = 0;
    pitch_sig = 0;
    yaw_sig   = 0.5f * (_parameter->ParameterStorage.list.min_yaw_aux + _parameter->ParameterStorage.list.max_yaw_aux);
  }

  #if HIL_SIM == 1 
  motor.chan1 = 1500 + roll_sig;
  motor.chan2 = 1500 + pitch_sig;
  motor.chan3 = throttle;  
  motor.chan4 = yaw_sig;  
  #else
  motor.chan1 = throttle + roll_sig + pitch_sig;
  motor.chan2 = throttle - roll_sig + pitch_sig;
  motor.chan3 = throttle - pitch_sig;  
  motor.chan4 = yaw_sig;  
  #endif
  
  /* 
   *  Write Servos
   */
  if(armed == true){
    motor_one.writeMicroseconds(motor.chan1);
    motor_two.writeMicroseconds(motor.chan2);
    motor_three.writeMicroseconds(motor.chan3);
    servo_yaw.writeMicroseconds(motor.chan4);
  }
  else
  {
    motor_one.writeMicroseconds(979);
    motor_two.writeMicroseconds(979);
    motor_three.writeMicroseconds(979);
    servo_yaw.writeMicroseconds(1500);
  }
}


void AP_Program::AltHold(servo_out &motor){

  uint32_t now       = millis();
  float    DT        = (now - _last_althold_msec) * 1e-3f;
  _last_althold_msec = now;


  if(DT > 1.0f)
  {
    DT                = 0.01f;
    arm_msec          = millis();
    disarm_msec       = millis();
    _alt_hold         = false;
    _altitude_to_hold = _height;
    rate_pid_throttle.reset();
    rate_pid_accel.reset();
  }

   
  if(_radio->chan4() > 1900     && _radio->chan3() < 1100){
    arm_msec   = millis();  
    if((arm_msec - disarm_msec) >= 2000){
      if(_ahrs->healthy())
      {
        armed  = true; 
      }
      disarm_msec = arm_msec;
    }
  }
  else if(_radio->chan3() < 1100)
  {
    disarm_msec = millis();
    if(armed  == false)
    {
     arm_msec  = millis(); 
    }    
    if((disarm_msec - arm_msec) >= 3000)
    {
      armed     = false;
      arm_msec  = disarm_msec;
    }
  }

  /*
   * Throttle Functions
   * SlewRate is the percentage throttle change over the full scale range in one second
   * SR = (ΔThrottle/FullScaleRange)/dt
   */
  float _throttle = _radio->chan3(); 
  _throttle       = constrain_float(_throttle, min_rec, max_rec - 50);
  float range_lim = max_rec - min_rec;
  float range     = _throttle - min_rec;
  float perc      = (range/range_lim) * 100;      

 
  /* 
   * populate servo channel  
   * get roll, pitch and heading stabilised
   */
  float roll_sig     = roll_stabilise();
  float pitch_sig    = pitch_stabilise();
  float yaw_sig      = yaw_stabilise();
  float throttle_sig = throttle_control();
  
  if(perc <= 10){
    rate_pid_roll.reset();
    rate_pid_pitch.reset();
    rate_pid_yaw.reset(); 
    rate_pid_throttle.reset();
    rate_pid_accel.reset();
    roll_sig  = 0;
    pitch_sig = 0;
    yaw_sig   = 0.5f * (_parameter->ParameterStorage.list.min_yaw_aux + _parameter->ParameterStorage.list.max_yaw_aux);
  }

  #if HIL_SIM == 1 
  motor.chan1 = 1500 + roll_sig;
  motor.chan2 = 1500 + pitch_sig;
  motor.chan3 = throttle_sig;  
  motor.chan4 = yaw_sig;   
  #else
  motor.chan1 = throttle_sig + roll_sig + pitch_sig;
  motor.chan2 = throttle_sig - roll_sig + pitch_sig;
  motor.chan3 = throttle_sig - pitch_sig;  
  motor.chan4 = yaw_sig;  
  
  #endif
    
  /* 
   *  Write Servos
   */
  if(armed == true){
    motor_one.writeMicroseconds(motor.chan1);
    motor_two.writeMicroseconds(motor.chan2);
    motor_three.writeMicroseconds(motor.chan3);
    servo_yaw.writeMicroseconds(motor.chan4);
  }
  else
  {
    motor_one.writeMicroseconds(979);
    motor_two.writeMicroseconds(979);
    motor_three.writeMicroseconds(979);
    servo_yaw.writeMicroseconds(1500);
  }  
}


float AP_Program::roll_stabilise(){
  rollchan              = _radio->chan1();
  float rollmid         = 0.5f * (_parameter->ParameterStorage.list.min_roll_aux + _parameter->ParameterStorage.list.max_roll_aux);
  float deadzone        = 40;
  float rollmin         = rollmid - deadzone;
  float rollmax         = rollmid + deadzone;
  if(rollchan >= rollmin && rollchan <= rollmax)
    rollchan  = rollmid;
  else if(rollchan < rollmin)
    rollchan  = rollchan + deadzone;
  else if(rollchan > rollmax)
    rollchan  = rollchan - deadzone;
    
  float rev_roll_sig    = _parameter->ParameterStorage.list.rev_roll_sig;
  float dem_roll        = map_float(rollchan, _parameter->ParameterStorage.list.min_roll_aux, _parameter->ParameterStorage.list.max_roll_aux, -_parameter->ParameterStorage.list.max_roll_deg, _parameter->ParameterStorage.list.max_roll_deg);
  if(fabs(rev_roll_sig) == 1.0f)
    dem_roll          *= rev_roll_sig;
  float _angle_err_r = dem_roll - ToDeg(_ahrs->roll);
  float _meas_rate_r = ToDeg(_ahrs->rollrate);  

  float roll_lim     = 450;
  float roll_sig     = 0;
  
  float desired_rate_r = stab_pid_roll.get_p(_angle_err_r);
  desired_rate_r       = constrain_float(desired_rate_r, -_parameter->ParameterStorage.list.roll_rmax, _parameter->ParameterStorage.list.roll_rmax);
  desired_rate_r       = ToRad(desired_rate_r);
  float rate_error_r   = desired_rate_r - _ahrs->rollrate;
  roll_sig             = map_float(rate_pid_roll.get_pid(rate_error_r), -1.0f, 1.0f, -roll_lim, roll_lim);
  return roll_sig;
}


float AP_Program::pitch_stabilise(){
  pitchchan      = _radio->chan2();
  float pitchmid = 0.5f * (_parameter->ParameterStorage.list.min_pitch_aux + _parameter->ParameterStorage.list.max_pitch_aux);
  float deadzone = 40;
  float pitchmin = pitchmid - deadzone;
  float pitchmax = pitchmid + deadzone;
  if(pitchchan  >= pitchmin && pitchchan <= pitchmax)
    pitchchan  = pitchmid;
  else if(pitchchan < pitchmin)
    pitchchan  = pitchchan + deadzone;
  else if(pitchchan > pitchmax)
    pitchchan  = pitchchan - deadzone;
  
  float  rev_pitch_sig = _parameter->ParameterStorage.list.rev_pitch_sig;
  float dem_pitch      = map_float(pitchchan, _parameter->ParameterStorage.list.min_pitch_aux, _parameter->ParameterStorage.list.max_pitch_aux, -_parameter->ParameterStorage.list.max_pitch_deg, _parameter->ParameterStorage.list.max_pitch_deg); 
  if(fabs(rev_pitch_sig) == 1.0f)
    dem_pitch *= rev_pitch_sig;
  float _angle_err_p  = dem_pitch - ToDeg(_ahrs->pitch);
  float _meas_rate_p  = ToDeg(_ahrs->pitchrate);  

  float pitch_lim     = 450;
  float pitch_sig     = 0;
  
  float desired_rate_p = stab_pid_pitch.get_p(_angle_err_p);
  desired_rate_p       = constrain_float(desired_rate_p, -_parameter->ParameterStorage.list.pitch_rmax, _parameter->ParameterStorage.list.pitch_rmax);
  desired_rate_p       = ToRad(desired_rate_p);
  float rate_error_p   = desired_rate_p - _ahrs->pitchrate;
  pitch_sig            = map_float(rate_pid_pitch.get_pid(rate_error_p), -1.0f, 1.0f, -pitch_lim, pitch_lim);

  return pitch_sig;
}


float AP_Program::yaw_stabilise(){
  float chan4     = _radio->chan4();
  float mid_yaw   =  0.5f * (_parameter->ParameterStorage.list.min_yaw_aux + _parameter->ParameterStorage.list.max_yaw_aux);
  float deadzone  = 50;
  float yawmin    = mid_yaw - deadzone;
  float yawmax    = mid_yaw + deadzone;
  
  if(chan4 >= yawmin && chan4 <= yawmax)
    chan4  = mid_yaw;
  else if(chan4 < yawmin)
    chan4  = chan4 + deadzone;
  else if(chan4 > yawmax)
    chan4  = chan4 - deadzone;
  
  float dem_rud   = map_float(chan4, _parameter->ParameterStorage.list.min_yaw_aux, _parameter->ParameterStorage.list.max_yaw_aux, -60, 60); 
  float yaw_error = 0;
  if(armed == true){
    if(dem_rud == 0)
    {
      if(_hdg_hold == false)
      {
         yaw_dem   = ToDeg(_ahrs->yaw);
         _hdg_hold = true;
      }          
      yaw_error = yaw_dem -  ToDeg(_ahrs->yaw);  
      if(yaw_error >  180)
      {
        yaw_error = yaw_error - 360;
      }  
      if(yaw_error < -180)
      {
        yaw_error = yaw_error + 360; 
      }    
    }
    else
    {
      yaw_error  = dem_rud;
      _hdg_hold = false;
    } 
  }
  else
  {
      yaw_dem   = ToDeg(_ahrs->yaw); // just keep updating the demanded yaw
  }

  float rev_rud_sig  = _parameter->ParameterStorage.list.rev_yaw_sig;
  if(fabs(rev_rud_sig) == 1.0f)
    yaw_error *= rev_rud_sig; 
  float _angle_err_y  = yaw_error; // replace this with yaw_error
  float _meas_rate_y  = ToDeg(_ahrs->yawrate); 

  float yaw_sig   = 1500;
 
  float desired_rate_y = stab_pid_yaw.get_p(_angle_err_y);
  desired_rate_y       = constrain_float(desired_rate_y, -_parameter->ParameterStorage.list.steer_rmax, _parameter->ParameterStorage.list.steer_rmax);
  desired_rate_y       = ToRad(desired_rate_y);
  float rate_error_y   = desired_rate_y - _ahrs->yawrate;
  yaw_sig              = map_float(rate_pid_yaw.get_pid(rate_error_y), -1.0f, 1.0f, _parameter->ParameterStorage.list.min_yaw_aux, _parameter->ParameterStorage.list.max_yaw_aux);

  return yaw_sig;
}

float AP_Program::throttle_rate(){
  throttle          = _radio->chan3(); 
  throttle          = constrain_float(throttle, min_rec, max_rec - 50);
  float range_lim   = max_rec - min_rec;
  float range       = throttle - min_rec;
  
  float perc        = (range/range_lim) * 100;      
  float max_range   = (max_rec  - min_rec);
  float hover_range = 0.60 * max_range;
  float hover_thr   = min_rec + hover_range;

  float upper_limit = max_rec - hover_thr;
  float lower_limit = hover_thr - min_rec;

  float fraction    = 0.13;
  float dead_range  = fraction * max_range;
  float hovermin    = hover_thr - dead_range; // 13% less
  float hovermax    = hover_thr + dead_range; // 13% more

  float throttle_out   = 0;
  float _demanded_rate = 0;
  if(throttle >= hovermin && throttle <= hovermax)
  {
    // altitude hold
    //throttle  = mid_yaw;
    if(_alt_hold == false){
      _alt_hold         = true;   
      _altitude_to_hold = _height;
    }

    float _target_altitude = _altitude_to_hold;
    float _imu_altitude    = _height;

    float _altitude_error  = (_target_altitude - _imu_altitude) * 100; // cm/s
    float THR_ALT_P        = 1;
     
    _demanded_rate   = _altitude_error * THR_ALT_P;                    
  }
  else if(throttle < hovermin)
  {
    // descend    
    _alt_hold      = false;
    _demanded_rate = map_float(throttle, min_rec, hovermin, -250, 0); // cm/s
  }
  else if(throttle > hovermax)
  {
    // climb
    _alt_hold      = false;
    _demanded_rate = map_float(throttle, hovermax, max_rec, 0, 250);  // cm/s
  }

  // calculate rate error
  // rate_error is in cm/s/s
  _demanded_rate    = constrain_float(_demanded_rate, -250, 250);    // cm/s
  float _imu_rate   = _climbrate * 100;                              // cm/s
  float _rate_error = _demanded_rate - _imu_rate;                    // cm/s

  
  // 50Hz ThrottleRate PID
  // rate error to acceleration demand
  // demanded acceleration is in earh frame
  // transform to body frame for imu comparison
  uint64_t tnow = micros();
  float DT      = (tnow - _throttle_50Hz_usec) * 1e-6f; 
  if(DT >= 0.02)
  {
    _throttle_50Hz_usec = tnow;
    if(DT >= 1.0f){
    rate_pid_throttle.reset();
    rate_pid_accel.   reset();
    }       
    rate_pid_throttle.set_pid(6, 0,0,500);
    float _thr_accel = rate_pid_throttle.get_pid(_rate_error);      // limited to -500cm/s/s to 1500cm/s/s
    vector3f _demanded_accel_earth(0,0,_thr_accel);                 // demanded acceleration in the earth frame
    _demanded_accel_body   = _ahrs->dcm().mul_transpose(_demanded_accel_earth);
    _demanded_accel_body.x = _demanded_accel_body.y = 0;            // cm/s/s
  }


  // 100Hz ThrottleAcceleration PID
  // Acceleration error to throttle output
  // Correct gravity term to body frame for accelerometer correction
  rate_pid_accel.set_pid(0.75, 1.5, 0.0,500);
  float _gravity_ef     =  9.8065;                                                      // m/s/s
  float _gravity_body   =  (_ahrs->dcm().mul_transpose(vector3f(0,0,_gravity_ef))).z;   // m/s/sS
  float _imu_accel_body = -(_ahrs->acc.z + _gravity_body) * 100;                        // cm/s/s
  float _accel_error    =  _demanded_accel_body.z - _imu_accel_body;                    // cm/s/s
  float _thr_out        = map_float(rate_pid_accel.get_pid(_accel_error), -2000, 2000, -lower_limit, upper_limit);     // throttle units constrained to +/- 500

  throttle_out  = hover_thr + _thr_out;

  /*
  Serial.print(_demanded_rate);
  Serial.print("  ");
  Serial.print(_thr_out);
  Serial.print("  ");
  Serial.println(throttle_out);
  */
 
  
  return throttle_out;
}

float AP_Program::throttle_control(){
  throttle          = _radio->chan3(); 
  throttle          = constrain_float(throttle, min_rec, max_rec - 50);
  float range_lim   = max_rec - min_rec;
  float range       = throttle - min_rec;
  
  float perc        = (range/range_lim) * 100;      
  float max_range   = (max_rec  - min_rec);
  float hover_range = 0.6 * max_range;
  float hover_thr   = min_rec + hover_range;

  float upper_limit = max_rec - hover_thr;
  float lower_limit = hover_thr - min_rec;

  float fraction    = 0.15;
  float dead_range  = fraction * max_range;
  float hovermin    = hover_thr - dead_range; // 13% less
  float hovermax    = hover_thr + dead_range; // 13% more

  float throttle_out          = 0;
  float _demanded_rate        = 0;
  float _altitude_error       = 0;
  static float errorAltitudeI = 0;
  float PALT                  = 6.4;
  float IALT                  = 0.025;
  float DALT                  = 24;
  float PVEL                  = 12.0; 
  float IVEL                  = 0.045;
  float DVEL                  = 1;
  static float AltPID         = 0;
  static float lastVelError   = 0;
  static float tau            = 0.50;
  static float output         = 0;
  
  if(throttle >= hovermin && throttle <= hovermax)
  {
    // altitude hold
    //throttle  = mid_yaw;
    if(_alt_hold == false){
      _alt_hold            = true;   
      _altitude_to_hold    = _height;
    }

    float _target_altitude = _altitude_to_hold;
    float _imu_altitude    = _height;

    float _altitude_error  = (_target_altitude - _imu_altitude) * 100;       // in decintimeter
    //_altitude_error        = constrain_float(_altitude_error, -250, 250);    // constrain to 10metres equivalent to 100 decimeter error        
    _demanded_rate         = _altitude_error/tau;                            // cm/s           
  }
  else if(throttle < hovermin)
  {
    // descend    
    _alt_hold         = false;  
    _demanded_rate    = map_float(throttle, min_rec, hovermin, -250, 0); // cm/s
  }
  else if(throttle > hovermax)
  {
    // climb
    _alt_hold        = false;
    _demanded_rate   = map_float(throttle, hovermax, max_rec, 0, 250);  // cm/s
  }

  // calculate rate error
  // rate_error is in cm/s/s
  _demanded_rate    = constrain_float(_demanded_rate, -250, 250);    // cm/s
 
  float _imu_rate   = _climbrate * 100;                              // cm/s  
  float _rate_error = _demanded_rate - _imu_rate;                    // cm/s


  // Throttle Integrator Calculations
  uint64_t tnow       = micros();
  float DT            = (tnow - _throttle_50Hz_usec) * 1e-6f; 
  _throttle_50Hz_usec = tnow; 
  if(DT >= 1.0f)
  {
    thr_integrator      = 0;
    _derivative         = 0;
    _last_demanded_rate = 0;

    errorAltitudeI      = 0;
  }
 
  static float thr_ki = 2.0f;
  static float thr_kp = 1.0f;
  static float thr_kd = 0.4f;


  // Integrator
  float ITerm = 0;
  if(thr_ki   > 0)
  {
    ITerm    = thr_ki * _demanded_rate * DT;
    if(output > 150){
      ITerm  = min(ITerm, 0);
    }
    else if(output < -150){
      ITerm = max(ITerm, 0);
    }
    thr_integrator += ITerm;
    if(DT >= 1.0f){
      thr_integrator = 0;
    }
  }


  // Derivative
  /*
  if(thr_kd > 0)
  {
    static float _last_rate_error = 0;

    if(DT > 1.0f){
      _last_rate_error = 0;
      _derivative      = 0;
      DT               = 0.01f;
    }
    float derivative   = (_rate_error - _last_rate_error)/DT;
    _last_rate_error   = _rate_error;

    // LPF
    float FC    = 20;
    float RC    = 1/ (2 * PI * FC);
    float alpha = DT/(DT + RC);

    // derivative term
    _derivative = _derivative + (derivative - _derivative) * alpha;
  }
` */

  float D = thr_kd * _rate_error;                       // Rate
  
  
  float I = constrain_float(thr_integrator, -100, 100); // Altitude
  float P = thr_kp * _demanded_rate;                    // Altitude

  output  = P + I + D;
  output  = constrain_float(output, -150, 150);

  throttle_out = throttle + output;
 

  /*
  //**** Alt. Set Point stabilization PID ****
  // error = constrain((AltHold - EstAlt)*10, -100, 100); //  +/-10m,  1 decimeter accuracy
  errorAltitudeI += _altitude_error;
  errorAltitudeI  = constrain(errorAltitudeI,-5000,5000);
      
  float PTerm  = PALT * _altitude_error/100;             // 16 bits is ok here
  float ITerm  = (int32_t)IALT*errorAltitudeI/4000;      //      
  AltPID       = PTerm + ITerm;

  //**** Velocity stabilization PD ****        
  float error  = constrain(_climbrate*2000.0f, -30000.0f, 30000.0f);
  float delta  = error - lastVelError;
  lastVelError = error;

  PTerm        = (int32_t)error * PVEL/800;
  float DTerm  = (int32_t)delta * DVEL/16;
       
  throttle_out = throttle + constrain(AltPID - (PTerm - DTerm),-100,+100);
  */
  
  return throttle_out;
}

