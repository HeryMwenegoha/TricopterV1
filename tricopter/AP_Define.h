#pragma once
#include "Arduino.h"

#define HIL_SIM         0
#define mainloop_rate   20
#define UAV             100  // 100 - 200 COPTERS  
#define GCS             255  // Ground Control Station
#define MSP             254  // Mission Planning

typedef short      i16;
typedef unsigned short u16;
typedef long i32;
typedef unsigned long u32;

//static  constexpr uint8_t UAV_ID  = 1;     // Manually Set So that GCS only transmits and receives to this UAV ID
static  boolean SEND_MAV_DATA       = false;     // Set False to Stop MAV from sending messages before parameters are sent, making sure the serialport is free

static uint32_t mainloop_lastTime  = 0;
static uint32_t loop_lapse_time    = 0;     // current time elapsed since start of void loop...
byte            led_counter = 0;


enum MODE {PREFLIGHT, MANUAL, STABILISE, ALTHOLD, IGNORE};

byte FLIGHT_MODE(const uint16_t chan8)
{
   if(chan8 > 1600 && chan8 < 2000)
   return MANUAL;
   else if(chan8 > 1300 && chan8 < 1600)
   return STABILISE;
   else if(chan8 > 1000 && chan8 < 1300)
   return ALTHOLD;
   else
   return IGNORE;
}

struct servo_out
{
  uint16_t chan1;
  uint16_t chan2;
  uint16_t chan3;
  uint16_t chan4;
  uint16_t chan5;
  uint16_t chan6;
  uint16_t chan7;
  uint16_t chan8;
  uint16_t throttle(){return (chan3 - 1000)/10;};
}_servo_out;



/*@ MPU9150 Sensor 
 */
#define WHO_AM_I_REG     0x75
#define MPU_ID           0x68

#define PWR_MGMT1_REG    0x6B
#define PWR_MGMT1_VAL    0x03  // selects the Z gyro as the clock source and disable temperature

#define USER_CTRL_REG    0x6A 
#define USER_CTRL_VAL    0x01  // reset signal path and sensor registers

#define SMPLRT_DIV_REG   0x19  // REGISTER
#define SMPLRT_DIV_VAL   0x13  // Gives a sample rate of 50Hz

#define DLPF_CONFIG_REG  0x1A  // REGISTER
#define DPLF_CONFIG_VAL  0x04  // 20Hz DLPF

#define GYRO_CONFIG_REG  0x1B  // REGISTER
#define GYRO_CONFIG_500  0x08  // 500DPS

#define ACCEL_CONFIG_REG 0x1C  // REGISTER
#define ACCEL_CONFIG_VAL 0x08  // DHPF None +/- 4G | 8192LSB/mg

#define INT_ENABLE_REG   0x38  // REGISTER
#define INT_ENABLE_VAL   0x01  // Data Ready En

// Accels 4G's 8192 LSB/MG
#define ACCEL_XOUT_H     0x3B
#define ACCEL_XOUT_L     0x3C
#define ACCEL_YOUT_H     0x3D
#define ACCEL_YOUT_L     0x3E
#define ACCEL_ZOUT_H     0x3F
#define ACCEL_ZOUT_L     0x40

// Gyros 500DPS 65.5 LSB/DEG/SEC
#define GYRO_XOUT_H      0x43
#define GYRO_XOUT_L      0x44
#define GYRO_YOUT_H      0x45
#define GYRO_YOUT_L      0x46
#define GYRO_ZOUT_H      0x47
#define GYRO_ZOUT_L      0x48

// Magnetometer
#define DEVICE_ID        0x48
#define STATUS_REG       0x02
#define MAG_X_OUT_L      0x03
#define MAG_X_OUT_H      0x04
#define MAG_Y_OUT_L      0x05
#define MAG_Y_OUT_H      0x06
#define MAG_Z_OUT_L      0x07
#define MAG_Z_OUT_H      0x08
#define CNTL_REG         0x0A
#define CNTL_VAL         0x01 // Single measuremnet
