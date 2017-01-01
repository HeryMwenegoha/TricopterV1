#pragma once
#include "AP_Program.h"
#define HEADER_LEN      8

class AP_Mavlink
{  
  private:
  HardwareSerial* Port;
  AP_Storage      &_AP_params;
  AP_AHRS         &_AP_ahrs;
  AP_GPS          &_AP_gps;
  AP_INS          &_AP_ins;
  AP_Compass      &_AP_compass;
  AP_Airspeed     &_AP_airspeed;
  AP_Baro         &_AP_baro;
  AP_WayPoint     &_AP_waypoint;
  AP_Program      &_AP_program;
  AP_HgtFilter_AQ &_AP_hgtfilter_aq;
  AP_HgtFilter    &_AP_hgtfilter;
  AP_BatteryVolts &_AP_batteryvolts;
  
  void stream_10Hz();
  void stream_3Hz();
  void stream_1Hz(); 
  byte _stream_speed_10Hz;
  byte _stream_speed_3Hz;
  byte _stream_speed_1Hz;

  public:
  AP_Mavlink(AP_Storage &ap_st, AP_AHRS &ap_ah, AP_GPS &ap_gp,
  AP_INS &ap_in, 
  AP_Compass &ap_co, 
  AP_Airspeed &ap_ar,
  AP_Baro &ap_ba, 
  AP_WayPoint &ap_wp, 
  AP_Program  &ap_pgm,
  AP_HgtFilter_AQ &ap_hgtfil,
  AP_HgtFilter  &_ap_hgt,
  AP_BatteryVolts &_ap_bat
  ):
  _AP_params(ap_st),
  _AP_ahrs(ap_ah),
  _AP_gps(ap_gp),
  _AP_ins(ap_in),
  _AP_compass(ap_co),
  _AP_airspeed(ap_ar),
  _AP_baro(ap_ba),
  _AP_program(ap_pgm),
  _stream_speed_10Hz(0),
  _stream_speed_3Hz(0),
  _stream_speed_1Hz(0),
  _AP_waypoint(ap_wp),
  _AP_hgtfilter_aq(ap_hgtfil),
  _AP_hgtfilter(_ap_hgt),
  _AP_batteryvolts(_ap_bat)
  {};

  void initialise(HardwareSerial *_Port)
  {
    Port = _Port;
  }

  void sendstream(); // execute in a 50Hz loop
  
  void readstream()
  {
   mavlink_message_t msg;
   mavlink_status_t status;
   while(Port->available())
   {
    uint8_t read_bytes = Port->read();  
    if(mavlink_parse_char(MAVLINK_COMM_0, read_bytes, &msg, &status))
    {
      switch(msg.msgid)
      {
         case MAVLINK_MSG_ID_HEARTBEAT:{         
            }
            break;

          /*
           * Receiiving Mission Count and then Sending Mission Items
           */
         case MAVLINK_MSG_ID_MISSION_COUNT:{
             if(mavlink_msg_mission_count_get_target_system(&msg) == UAV){
                _AP_waypoint.count.total = static_cast<AP_u8>(mavlink_msg_mission_count_get_count(&msg));   // need to save it
                _AP_waypoint.count.save();
                send_mission_request(0);
              }
             }
             break;
         
         case MAVLINK_MSG_ID_MISSION_ITEM:{         
           if(mavlink_msg_mission_item_get_target_system(&msg) == UAV){           
              uint16_t _seq = mavlink_msg_mission_item_get_seq(&msg);          
              _AP_waypoint.update(mavlink_msg_mission_item_get_x(&msg), mavlink_msg_mission_item_get_y(&msg),static_cast<AP_u16>(mavlink_msg_mission_item_get_z(&msg)),static_cast<AP_u8>(mavlink_msg_mission_item_get_param1(&msg)),static_cast<AP_u8>(_seq),static_cast<AP_u8>(mavlink_msg_mission_item_get_command(&msg)));                    
              _AP_waypoint.write(_seq);
              
              if(_seq < (_AP_waypoint.count.total - 1)){
                send_mission_request(_seq + 1);
              }else if(_seq == (_AP_waypoint.count.total - 1)){    
                send_mission_ack(MAV_MISSION_ACCEPTED);      
              }else if(_seq >= MAX_WAYPOINTS){
                 send_mission_ack(MAV_MISSION_NO_SPACE);                 
              }else{
                send_mission_ack(MAV_MISSION_INVALID);      
              }          
             }
           }
           break;

        /* 
         *  Sending Requested Mission Item List
         */
         case MAVLINK_MSG_ID_MISSION_REQUEST_LIST:{
             if(mavlink_msg_mission_request_list_get_target_system(&msg) == UAV)
                send_mission_count(_AP_waypoint.count.total);
             }
             break;

         case MAVLINK_MSG_ID_MISSION_REQUEST:{
             if(mavlink_msg_mission_request_get_target_system(&msg) == UAV)
                send_mission_item(static_cast<AP_u8>(mavlink_msg_mission_request_get_seq(&msg)));  
             }       
             break;
        
         case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:{
             SEND_MAV_DATA  = true;
             _AP_params.SendParamList(UAV);
             }
             break;
         
         case MAVLINK_MSG_ID_PARAM_SET:{
             if(mavlink_msg_param_set_get_target_system(&msg) == UAV)
             {
               char param_id[16];
               float param_value = mavlink_msg_param_set_get_param_value(&msg);
               mavlink_msg_param_set_get_param_id(&msg, param_id);
               _AP_params.UpdateStorage(UAV, param_id, param_value);
             }
             }
             break;
         
         case MAVLINK_MSG_ID_HIL_STATE:
         {
         #if HIL_SIM == 1
         hil_ahrs _hil_ahrs;
         _hil_ahrs.roll     = mavlink_msg_hil_state_get_roll(&msg);      // rad
         _hil_ahrs.pitch    = mavlink_msg_hil_state_get_pitch(&msg);     // rad
         _hil_ahrs.yaw      = mavlink_msg_hil_state_get_yaw(&msg);       // rad
         _hil_ahrs.rollrate = mavlink_msg_hil_state_get_rollspeed(&msg); // rad/sec
         _hil_ahrs.pitchrate= mavlink_msg_hil_state_get_pitchspeed(&msg);// rad/sec
         _hil_ahrs.yawrate  = mavlink_msg_hil_state_get_yawspeed(&msg);  // rad/sec        
         _hil_ahrs.xacc     = (mavlink_msg_hil_state_get_xacc(&msg) * 0.001f) * 9.81f;       // mg to m/s2
         _hil_ahrs.yacc     = (mavlink_msg_hil_state_get_yacc(&msg) * 0.001f) * 9.81f;       // mg to m/s2
         _hil_ahrs.zacc     = (mavlink_msg_hil_state_get_zacc(&msg) * 0.001f) * 9.81f;       // mg to m/s2
        
         /* 
          hil_gps _hil_gps;         
         _hil_gps.location.x= static_cast<float>(mavlink_msg_hil_state_get_lat(&msg) * 1e-7);
         _hil_gps.location.y= static_cast<float>(mavlink_msg_hil_state_get_lon(&msg) * 1e-7);
         _hil_gps.velocity.x= static_cast<float>( mavlink_msg_hil_state_get_vx(&msg) * 1e-2);
         _hil_gps.velocity.y= static_cast<float>( mavlink_msg_hil_state_get_vy(&msg) * 1e-2);
         _hil_gps.velocity.z= static_cast<float>( mavlink_msg_hil_state_get_vz(&msg) * 1e-2);
         _hil_gps.altitude  = static_cast<float>(mavlink_msg_hil_state_get_alt(&msg) * 1e-3); 
         _hil_gps.time_usec = mavlink_msg_hil_state_get_time_usec(&msg);
         */
         float lat = static_cast<float>(mavlink_msg_hil_state_get_lat(&msg) * 1e-7);
         float lon = static_cast<float>(mavlink_msg_hil_state_get_lon(&msg) * 1e-7);
         float vx  = static_cast<float>( mavlink_msg_hil_state_get_vx(&msg) * 1e-2);
         float vy  = static_cast<float>( mavlink_msg_hil_state_get_vy(&msg) * 1e-2);
         float vz  = static_cast<float>( mavlink_msg_hil_state_get_vz(&msg) * 1e-2);
         float alt = static_cast<float>(mavlink_msg_hil_state_get_alt(&msg) * 1e-3); 
                  
         _AP_ahrs.setHil();        
         _AP_ins.setHil(_hil_ahrs.rollrate,_hil_ahrs.pitchrate, _hil_ahrs.yawrate,_hil_ahrs.xacc,_hil_ahrs.yacc,_hil_ahrs.zacc);        
         _AP_compass.setHil(_hil_ahrs.roll, _hil_ahrs.pitch, _hil_ahrs.yaw);
         _AP_gps.setHil(lat, lon, alt, vector3f(vx, vy, vz));
         _AP_baro.setHil(alt);
 
         //Serial.println(millis());
         #endif  
         }
         break;
           
         case MAVLINK_MSG_ID_VFR_HUD:{
             #if HIL_SIM == 1
             _AP_airspeed.setHil(mavlink_msg_vfr_hud_get_airspeed(&msg));
             _AP_gps.setHilHeading(mavlink_msg_vfr_hud_get_heading(&msg));  // 0 -360 in degress
             #endif
             }
             break;
         
         case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE:{
             _servo_out.chan5 = mavlink_msg_rc_channels_override_get_chan5_raw(&msg);
             _servo_out.chan6 = mavlink_msg_rc_channels_override_get_chan6_raw(&msg);
             }
             break;
      }
    }
   }
 }
 
 void send_hb(boolean &Send_Allowed)
 {   
     mavlink_message_t  msg_t;
     uint8_t  buf[MAVLINK_MSG_ID_HEARTBEAT_LEN + HEADER_LEN];     // Pax 9 + 8 = 17bytes
     
     mavlink_msg_heartbeat_pack(
     UAV, MAV_COMP_ID_ALL, &msg_t,
     MAV_TYPE_FIXED_WING, 
     MAV_AUTOPILOT_GENERIC,
     _AP_ahrs.get_flightmode(),    // base mode
     MAV_MODE_PREFLIGHT,          // custom mode
     MAV_STATE_STANDBY);
     
     uint16_t len = mavlink_msg_to_send_buffer(buf,&msg_t); 
     
     if(Send_Allowed){
         Port->write(buf,len);
     }
 }

 void send_euler(uint32_t &loop_lapse, boolean &Send_Allowed)
 {   
     mavlink_message_t  msg_t;
     uint8_t  buf[MAVLINK_MSG_ID_ATTITUDE_LEN + HEADER_LEN];     // PAX 28 + 8 = 36bytes
     
     mavlink_msg_attitude_pack(
     UAV, MAV_COMP_ID_IMU, &msg_t,
     loop_lapse, 
     _AP_ahrs.roll, 
     _AP_ahrs.pitch, 
     _AP_ahrs.yaw,  // true heading as calculated by dcm
     _AP_ahrs.rollrate, 
     _AP_ahrs.pitchrate, 
     _AP_ahrs.yawrate);
   
     uint16_t len = mavlink_msg_to_send_buffer(buf,&msg_t); 
     
     if(Send_Allowed){
         Port->write(buf,len);
     }
 }


 void send_imuraw(uint32_t &loop_lapse, boolean &Send_Allowed)
 {   
     mavlink_message_t  msg_t;
     uint8_t  buf[MAVLINK_MSG_ID_RAW_IMU_LEN + HEADER_LEN];     // PAX 26 + 8 = 34 bytes 
     
     mavlink_msg_raw_imu_pack(
     UAV, MAV_COMP_ID_IMU, &msg_t,
     loop_lapse,
     static_cast<int16_t>(_AP_ins.raw_accel()[0].x), 
     static_cast<int16_t>(_AP_ins.raw_accel()[0].y),
     static_cast<int16_t>(_AP_ins.raw_accel()[0].z), 
     static_cast<int16_t>(_AP_ins.raw_gyro()[0].x), 
     static_cast<int16_t>(_AP_ins.raw_gyro()[0].y), 
     static_cast<int16_t>(_AP_ins.raw_gyro()[0].z), 
     static_cast<int16_t>(_AP_compass.raw_field().x), 
     static_cast<int16_t>(_AP_compass.raw_field().y), 
     static_cast<int16_t>(_AP_compass.raw_field().z)); 
     
     uint16_t len = mavlink_msg_to_send_buffer(buf,&msg_t); 
     
     if(Send_Allowed){
         Port->write(buf,len);
     }
 }

 void send_gps(uint32_t &loop_lapse,  boolean &Send_Allowed)
 {
     mavlink_message_t  msg_t;
     uint8_t  buf[MAVLINK_MSG_ID_GPS_RAW_INT_LEN + HEADER_LEN];     //  PAX 30 + 8 = 38 bytes
     mavlink_msg_gps_raw_int_pack(
     UAV, MAV_COMP_ID_GPS, &msg_t,
     loop_lapse, 
     _AP_gps.status(), 
     static_cast<i32>(_AP_gps.location().lat*1e7), 
     static_cast<i32>(_AP_gps.location().lon*1e7),
     static_cast<i32>(_AP_gps.altitude()    *1e3), 
     static_cast<u16>(_AP_gps.horizontal_dilution()),
     static_cast<u16>(_AP_gps.vertical_dilution()),
     static_cast<u16>(_AP_gps.groundspeed()  *1e2),
     static_cast<u16>(_AP_gps.heading()      *1e2), // COG
     _AP_gps.num_sats());
     
     uint16_t len = mavlink_msg_to_send_buffer(buf,&msg_t); 
     
     if(Send_Allowed){
         Port->write(buf,len);
     }
 }

 void send_servo(uint32_t &loop_lapse, servo_out *servoOut, boolean &Send_Allowed)
 {
   mavlink_message_t msg_t;
   uint8_t  buf[MAVLINK_MSG_ID_SERVO_OUTPUT_RAW_LEN + HEADER_LEN];     //  PAX 21 + 8 = 29 bytes
   
   mavlink_msg_servo_output_raw_pack(
   UAV, MAV_COMP_ID_ALL, &msg_t,
   loop_lapse     , 
   1              ,
   servoOut->chan1,  // roll
   servoOut->chan2,  // pitch
   servoOut->chan3,  // throttle
   servoOut->chan4,  // yaw
   servoOut->chan5,  // empty for override 
   servoOut->chan6,  // empty for overide
   servoOut->chan7,  // empty for override
   servoOut->chan8); // chan 8 - flight mode channel
   
   uint16_t len = mavlink_msg_to_send_buffer(buf,&msg_t); 
   //Serial.print("Aux "); Serial.print(servoOut->chan1); Serial.print(servoOut->chan2); Serial.print(servoOut->chan3); Serial.println(servoOut->chan4);
     
   if(Send_Allowed){
         Port->write(buf,len);
   }
 }
 
 /*@ Send vfr information
  */
 void send_vfr(uint32_t &loop_lapse, servo_out *srv_chan, boolean &Send_Allowed)
 {
   mavlink_message_t msg_t;
   uint8_t  buf[MAVLINK_MSG_ID_VFR_HUD_LEN + HEADER_LEN];     //  PAX 21 + 8 = 29 bytes
   
   mavlink_msg_vfr_hud_pack(
   UAV,
   MAV_COMP_ID_ALL,
   &msg_t,
   _AP_ahrs.airspeed_estimate(), 
   _AP_airspeed.get_airspeed(), 
   _AP_gps.heading()   , 
   srv_chan->throttle(),
   #if !AQ_FILTER
   _AP_hgtfilter_aq.height(),
   _AP_hgtfilter_aq.climbrate()
   #else
   _AP_hgtfilter.height(),
   _AP_hgtfilter.climbrate()
   #endif
   );
   uint16_t len = mavlink_msg_to_send_buffer(buf,&msg_t); 
        
   if(Send_Allowed){
         Port->write(buf,len);
   }
 }

 void send_status(uint32_t &loop_lapse,  boolean &Send_Allowed)
 {
   mavlink_message_t msg_t;
   uint8_t  buf[MAVLINK_MSG_ID_SYS_STATUS_LEN + HEADER_LEN];     //  PAX 21 + 8 = 29 bytes
   
   uint32_t onboard_control_sensors_present = B00111111; // Current controls
   uint32_t onboard_control_sensors_enabled = B00111111; // All sensors enabled
   uint32_t onboard_control_sensors_health  = B00111111; // All sensors healthy
   uint16_t load               = 50;                                   // Maximum usage in percentage of mainloop time
   uint16_t voltage_battery    = _AP_batteryvolts.get_adc() * _AP_params.ParameterStorage.list.PowerModule_Gain * 1000;
   int8_t   battery_remaining  = 0;
   uint16_t drop_rate_comm     = 0; 
   uint16_t errors_comm        = 0;
   uint16_t errors_count1      = _AP_ins.healthy(); // healthy 1
   uint16_t errors_count2      = _AP_gps.healthy(); // healthy 1
   uint16_t errors_count3      = _AP_baro.healthy();// healthy 1
   uint16_t errors_count4      = 0;
   
   mavlink_msg_sys_status_pack(
   UAV, 
   MAV_COMP_ID_ALL, 
   &msg_t,
   onboard_control_sensors_present,
   onboard_control_sensors_enabled, 
   onboard_control_sensors_health, 
   load, 
   voltage_battery,
   -1, // no current measurment
   battery_remaining, 
   drop_rate_comm, 
   errors_comm, 
   errors_count1,
   errors_count2,
   errors_count3, 
   errors_count4);
   
   uint16_t len = mavlink_msg_to_send_buffer(buf,&msg_t); 
        
   if(Send_Allowed){
    Port->write(buf,len);   
   }
 }


 void send_mission_request(const AP_u8 &seq)
 {
   mavlink_message_t msg_t;
   uint8_t  buf[MAVLINK_MSG_ID_MISSION_REQUEST_LEN + HEADER_LEN];     //  PAX 4 + 8 = 12 bytes
    
   mavlink_msg_mission_request_pack(
   UAV, 
   MAV_COMP_ID_ALL, 
   &msg_t,
   GCS, 
   MSP, 
   seq);   
   uint16_t len = mavlink_msg_to_send_buffer(buf,&msg_t);         
   Port->write(buf,len);   
 }

 void send_mission_ack(const AP_u8 mav_ack)
 {
   mavlink_message_t msg_t;
   uint8_t  buf[MAVLINK_MSG_ID_MISSION_ACK_LEN + HEADER_LEN];     //  PAX 3 + 8 = 11 bytes

   mavlink_msg_mission_ack_pack(
   UAV, 
   MAV_COMP_ID_ALL, 
   &msg_t,
   GCS, 
   MSP, 
   mav_ack);
     
   uint16_t len = mavlink_msg_to_send_buffer(buf,&msg_t);         
   Port->write(buf,len);   
 }

 void send_mission_count(const AP_u8 count)
 {
   mavlink_message_t msg_t;
   uint8_t  buf[MAVLINK_MSG_ID_MISSION_COUNT_LEN + HEADER_LEN];     //  PAX 4 + 8 = 11 bytes   
   mavlink_msg_mission_count_pack(
   UAV, 
   MAV_COMP_ID_ALL, 
   &msg_t,
   GCS, 
   MSP, 
   count);    
   uint16_t len = mavlink_msg_to_send_buffer(buf,&msg_t);            
   Port->write(buf,len);   
 }

 void send_mission_item(const AP_u8 seq)
 {
   if(seq >= MAX_WAYPOINTS)
    return;
  
   mavlink_message_t msg_t;
   uint8_t  buf[MAVLINK_MSG_ID_MISSION_ITEM_LEN + HEADER_LEN];     //  PAX 37 + 8 = 43 bytes   
   
   mavlink_msg_mission_item_pack(
   UAV, 
   MAV_COMP_ID_ALL, 
   &msg_t,
   GCS, 
   MSP, 
   _AP_waypoint.WayPoint[seq].seq, 
   MAV_FRAME_GLOBAL, 
   _AP_waypoint.WayPoint[seq].cmd, 
   0, 
   1, 
   _AP_waypoint.WayPoint[seq].rad, 
   255, 
   255, 
   255, 
   _AP_waypoint.WayPoint[seq].lat, 
   _AP_waypoint.WayPoint[seq].lon, 
   _AP_waypoint.WayPoint[seq].alt); 

   uint16_t len = mavlink_msg_to_send_buffer(buf,&msg_t);            
   Port->write(buf,len);   
 }
 
 
};
 
 void AP_Mavlink::sendstream()
 {
   stream_10Hz();

   #if HIL_SIM == 1
   send_servo(loop_lapse_time, &_servo_out, SEND_MAV_DATA);
   #endif
 }
 
 // 7Hz
 void AP_Mavlink::stream_10Hz()
 {
   switch(_stream_speed_10Hz)
   {
     case 0:
     _stream_speed_10Hz++;
     send_euler(loop_lapse_time, SEND_MAV_DATA);
     break;
     
     case 1:
     _stream_speed_10Hz++;
     break;
     
     case 2:
     _stream_speed_10Hz++;
     break;
     
     case 3:
     _stream_speed_10Hz++;
     send_vfr(loop_lapse_time, &_servo_out, SEND_MAV_DATA);
     break;
     
     case 4:
     _stream_speed_10Hz++;
     break;
     
     case 5:
     _stream_speed_10Hz++;
     send_imuraw(loop_lapse_time, SEND_MAV_DATA);
     break;
        
     case 6:
     stream_3Hz();
     _stream_speed_10Hz = 0;
     break;
   }   
 }

// 2Hz
 void AP_Mavlink::stream_3Hz()
 {
   switch(_stream_speed_3Hz)
   {
     case 0:
     _stream_speed_3Hz++;
     break;
     
     case 1:
     _stream_speed_3Hz++;
     #if HIL_SIM == 0
     send_servo(loop_lapse_time, &_servo_out, SEND_MAV_DATA);
     #endif
     break;
     
     case 2:
      _stream_speed_3Hz++;
      send_vfr(loop_lapse_time, &_servo_out, SEND_MAV_DATA);
     break;
     
     case 3:
     stream_1Hz();
     _stream_speed_3Hz = 0;
     break;
   }   
 }


 void AP_Mavlink::stream_1Hz()
 {
   switch(_stream_speed_1Hz)
   {
     case 0:
     _stream_speed_1Hz++;
     send_hb(SEND_MAV_DATA);
     break;
     
     case 1:
     _stream_speed_1Hz++;
     send_gps(loop_lapse_time,  SEND_MAV_DATA);
     break;
     
     case 2:
     _stream_speed_1Hz = 0;
     send_status(loop_lapse_time,  SEND_MAV_DATA); 
     break;
   }   
 }
