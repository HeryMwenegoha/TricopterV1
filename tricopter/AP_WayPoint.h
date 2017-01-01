#pragma once
#include "Arduino.h"
#include <EEPROM.h>

#define MAX_WAYPOINTS   10

class AP_WayPoint{
  public:
  AP_WayPoint(){
    count.total = 0;
    count.seq   = 0;
  };
  void initialise(); 
  
  struct _count{
    AP_u8  total;
    AP_u8  seq;        
    void   load(){  total = EEPROM.read(4000); }
    void   save(){  EEPROM.write(4000, total); }
  };
  _count count; 
  
  Location WayPoint[MAX_WAYPOINTS];
  
  void write(uint8_t _seq);
  bool read(uint8_t _seq,  Location &_wp);
  void update(float lat, float lon, AP_u16 alt, AP_u8 rad, AP_u8 seq, AP_u8 cmd);
};

void AP_WayPoint::initialise()
{
  count.load();

  if(count.total > MAX_WAYPOINTS){
    Serial.print(F("AP_WayPoint::initialise-WayPointCount"));
    Serial.print("\t");
    Serial.println(count.total);
    delay(10);
  }else{
    Serial.print(F("WayPoint Count: "));
    Serial.println(count.total);
    
    uint32_t to = millis();
    for(int i = 0; i < count.total; i++){
      read(i, WayPoint[i]);
      WayPoint[i].print();
    }
    uint32_t to2 = millis();
    
    Serial.print(F("Time Stamp: "));
    Serial.print(to2-to);
    Serial.println(F("ms"));
  }
}


void AP_WayPoint::write(uint8_t seq){
  union _wp_un{
    Location wp_struct;
    byte buffer[sizeof(Location)];
  };

  _wp_un wp_un;
  wp_un.wp_struct = WayPoint[seq];
  
  for(int i = 0; i < sizeof(Location); i++){
    EEPROM.write((i + (seq * sizeof(Location))) , wp_un.buffer[i]);  
  }
}

bool AP_WayPoint::read(uint8_t seq, Location &_wp)
{
  union _wp_un{
    Location wp_struct;
    byte buffer[sizeof(Location)];
  };
  
  _wp_un wp_un; 
  
  for(int i = 0; i < sizeof(Location); i++){
    wp_un.buffer[i] = EEPROM.read((i + (seq * sizeof(Location))));  
  } 
    
  if(wp_un.wp_struct.isvalid()){   
    _wp = wp_un.wp_struct;
    return true;
  }
  else {
    return false;
  }
}

void AP_WayPoint::update(float lat, float lon, AP_u16 alt, AP_u8 rad, AP_u8 seq, AP_u8 cmd)
{
  if(seq < MAX_WAYPOINTS){
  WayPoint[seq].lat = lat;
  WayPoint[seq].lon = lon;
  WayPoint[seq].alt = alt;
  WayPoint[seq].rad = rad;
  WayPoint[seq].seq = seq;
  WayPoint[seq].cmd = cmd;
  }
}

