
#include <Servo.h>

#include <TinyGPS.h> 
#include <Wire.h>
#include <avr/eeprom.h>

#define WP_10_BYTES 10
#define WP_START_BYTE 0x18
#define WP_COUNT 0x09
#define buzzer 7
#define radius 10

struct Location {
  float lat;
  float lng;
  long alt;
};

struct Location waypoint;

float GPS_HEADING;
byte num_WPTS;
byte wi=0;
int statemode=1;
byte set_heading_input; //0 = compass, 1 = GPS
int output;
int error;
int last_error;
int HMC5883Address = 0x1E;
int slaveAddress;
byte headingData[2];
int i, headingValue;
float flat, flon;
float heading;
int headinggps;
int buzzing;
Servo servo;
Servo esc;
TinyGPS gps;

byte speedesc= 80;
float x2lat;
float x2lon;
void gpsdump(TinyGPS &gps);
bool feedgps();
void printFloat(double f, int digits = 2);
char incoming[100];
byte in_index = 0;
byte decode_index = 0;



void setup(){
  slaveAddress = HMC5883Address >> 1;  
  Wire.begin();
  num_WPTS = eeprom_read_byte((uint8_t*)WP_COUNT);
  
  Serial.begin(115200);
  Serial1.begin(4800);
  
  pinMode(buzzer, OUTPUT);
  pinMode(2, INPUT);
  servo.attach(9);
  delay(1000);
  buzz();
}



void loop(){
  compassdata();    //update our heading
  if(get_gps_data()){  //update our location
    bearing_distanc_calc();  //find distance and bearing if new data is avialable
  }
  execute();     //make move based off of calculations
}
//-------------------------------------------------------------------------
//-------------------------------------------------------------------------

bool get_gps_data(){
  while(Serial1.available()){
    if(gps.encode(Serial1.read())){
      gpsdump(gps);
      return true;
    }
  }
  return false;
}

//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

void compassdata(){
  Wire.beginTransmission(slaveAddress);        //the wire stuff is for the compass module
  Wire.write("A");              
  Wire.endTransmission();
  delay(10);                  
  Wire.requestFrom(slaveAddress,2);       
  i = 0;
  while(Wire.available() && i < 2){ 
    headingData[i] = Wire.read();
    i++;
  }
  headingValue = headingData[0]*256 + headingData[1];
  int pracheading = headingValue / 10;      // this is the heading of the compass
  if(pracheading>0){
    headinggps=pracheading;
  }
}

//--------------below is the part of the code where everything is calculated--
//----------------------------------------------------------------------------

void bearing_distanc_calc(){
  waypoint= get_loc_with_index(wi); 
  x2lat=waypoint.lat;
  x2lon=waypoint.lng;

  float flat1=flat;            
  float flon1=flon;
  float dist_calc=0;
  float dist_calc2=0;
  float diflat=0;
  float diflon=0;

  //------------------------------------------ distance formula below
  diflat=radians(x2lat-flat1);  //notice it must be done in radians
  flat1=radians(flat1);
  x2lat=radians(x2lat);
  diflon=radians((x2lon)-(flon1));
  dist_calc = (sin(diflat/2.0)*sin(diflat/2.0));
  dist_calc2= cos(flat1);
  dist_calc2*=cos(x2lat);
  dist_calc2*=sin(diflon/2.0);                                       
  dist_calc2*=sin(diflon/2.0);
  dist_calc +=dist_calc2;
  dist_calc=(2*atan2(sqrt(dist_calc),sqrt(1.0-dist_calc)));
  dist_calc*=6371000.0; //Converting to meters

  int WPTS=num_WPTS;

  if(dist_calc<radius){
    while(wi+1==WPTS){ // we are done with the mission
      buzz();
      esc.write(80);
      delay(1000);
    }
  wi++;
  }
  
  //-----------------------------------------heading formula below
  flon1 = radians(flon1);  //also must be done in radians
  x2lon = radians(x2lon);
  heading = atan2(sin(x2lon-flon1)*cos(x2lat),cos(flat1)*sin(x2lat)-
  sin(flat1)*cos(x2lat)*cos(x2lon-flon1)),2*3.1415926535;
  heading = heading*180/3.1415926535;  // convert from radians to degrees
  int head =heading; 
  
  if(head<0){
    heading+=360;   //if the heading is negative then add 360 to make it positive
  }
}
  
//--------------------------------------------------------------------------
//--------------------------------------------------------------------------

void execute(){
  if(analogRead(1)>950){
    if(GPS_HEADING>0){
      headinggps=GPS_HEADING;
    }
  }
  error=(heading-headinggps);
  //handle wrap-around  
  if (error < -180){ 
    error = error + 360;  
  }
    
  if (error > 180){
    error = error - 360;
  }
  
  float HEADING_KP=.8;   //gains
  float HEADING_KD=.2; 
  int derror = ((error - last_error) / 2); 
  int ierror = ((error + last_error) / 2);   
  float diff = (HEADING_KP * error) + (HEADING_KD * derror);
  int PD_output=diff;
  last_error=error;
  servo.write(map(PD_output, -50, 50, 105, 75));
  esc.write(speedesc);  //drive the set speed
}

//--------------------------------------------------------------------------
//--------------------------------------------------------------------------

void buzz(){
  while(buzzing<500){
    digitalWrite(buzzer, HIGH);
    delayMicroseconds(250);
    digitalWrite(buzzer, LOW);
    delayMicroseconds(250);
    buzzing++;
  }
  delay(100);
  buzzing=0;
}

//--------------------------------------------------------------------------
//--------------------------------------------------------------------------

void printFloat(double number, int digits){
  // Handle negative numbers
  if (number < 0.0){
    Serial.print('-');
    number = -number;
  }

  // Round correctly so that print(1.999, 2) prints as "2.00"
  double rounding = 0.5;
  for (uint8_t i=0; i<digits; ++i){
    rounding /= 10.0;
    number += rounding;
  }

  // Extract the integer part of the number and print it
  unsigned long int_part = (unsigned long)number;
  double remainder = number - (double)int_part;
  Serial.print(int_part);
  // Print the decimal point, but only if there are digits beyond
  if (digits > 0){
    Serial.print("."); 
  // Extract digits from the remainder one at a time
    while (digits-- > 0){
      remainder *= 10.0;
      int toPrint = int(remainder);
      Serial.print(toPrint);
      remainder -= toPrint; 
    }
  } 
}

//--------------------------------------------------------------------------
//--------------------------------------------------------------------------

void gpsdump(TinyGPS &gps){
  long lat, lon;
  unsigned long age, date, time, chars;
  unsigned short sentences, failed;
  get_gps_data(); 
  gps.f_get_position(&flat, &flon, &age);
  get_gps_data();
  GPS_HEADING= gps.course();
  get_gps_data();
  gps.stats(&chars, &sentences, &failed);
}

//--------------------------------------------------------------------------
//--------------------------------------------------------------------------

struct Location get_loc_with_index(int ti){
  struct Location temp;
  long mem_position;
  mem_position = (long)(WP_START_BYTE + (ti) * WP_10_BYTES);
  eeprom_busy_wait();
  temp.lat = (float)((long)eeprom_read_dword((uint32_t*)mem_position)) / 1000000;
  mem_position += 4;
  eeprom_busy_wait();
  temp.lng = (float)((long)eeprom_read_dword((uint32_t*)mem_position)) / 1000000;
  mem_position += 4;
  temp.alt = 0;
  eeprom_busy_wait();
  temp.alt = (long)eeprom_read_word((uint16_t*)mem_position);
  return temp;
}





