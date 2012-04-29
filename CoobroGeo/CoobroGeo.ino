/*******************************************************************************
* Title: Coobro Geo
* Version: 1.01
* Date: 11-09-2011
* Company: Coobro Labs
* Website: http://CoobroLabs.com
* 
* Intro
* =====
* This is basic firmware for use with the Coobro Geo device which can be
* purchased at http://coobrolabs.com.  The most important thing you need to
* change in this code is the GPS coordinates of the 5 locations you wish
* to navigate to.  Please go to www.coobrolabs.com/contact if you need
* any help.
*
* This firmware wouldn't exist without help from the following individuals:
* ==========================================
* Mikal Hart (www.arduiniana.org)
* ==========================================
* Author of both the TinyGPS library, as well as the NewSoftSerial library.
* There is no doubt that the Coobro Geo would not exist if it wasn't for
* the efforts of this individual.
*
* ==========================================
* Sean Carney (www.seancarney.ca)
* ==========================================
* Special thanks to Sean for creating and documenting his Arduino GPS System.
* This was my original inspiration for creating the Coobro Geo.  His code was
* used to help figure out the best way to navigate to a GPS coordinate.
*
* Licenses
* ========
* The Coobro Geo hardware and firmware are released under the 
* Creative Commons Share Alike v3.0 license
* http://creativecommons.org/licenses/by-sa/3.0/ 
* You are free to take this piece of code, use it and modify it. 
* All we ask is attribution including the supporting libraries used in this 
* firmware. 
*
* Revision  Description
* ========  ===========
* 1.00      Initial public release.
* 1.01      Minor edits for compiling under Arduino 1.0.
*******************************************************************************/

#include <SoftwareSerial.h>
#include <TinyGPS.h>
#include <EEPROM.h>
#include "EEPROMAnything.h" // Required to save floats into EEPROM
#include <math.h>

TinyGPS gps;
SoftwareSerial nss(4, 3);

//--------------------------------------------------|
//                    WAYPOINTS                     |
//--------------------------------------------------|
//Please enter the latitude and longitude of your   |
//desired destinations:                              |
  #define GEO_LAT1                40.7809
  #define GEO_LON1               -77.998817
 
  #define GEO_LAT2                40.064287
  #define GEO_LON2               -105.210604
 
  #define GEO_LAT3                51.178418
  #define GEO_LON3               -1.826370
 
  #define GEO_LAT4                46.234020
  #define GEO_LON4                6.052672
 
  #define GEO_LAT5                37.2368
  #define GEO_LON5               -115.8102
//--------------------------------------------------|

// Current waypoint location
int currentWP = 0;
int currentBC = 0;

// Navigation location
float targetFlat;
float targetFlon;

// Last known position
float lastFlat = 0;
float lastFlon = 0;

// Trip distance
float tripDistance;

float geoLAT[] = {GEO_LAT1, GEO_LAT2, GEO_LAT3, GEO_LAT4, GEO_LAT5};
float geoLON[] = {GEO_LON1, GEO_LON2, GEO_LON3, GEO_LON4, GEO_LON5};

// Breadcrumbs location
struct config_t
{
  float BC_LAT1;
  float BC_LON1;
  float BC_LAT2;
  float BC_LON2;
  float BC_LAT3;
  float BC_LON3;
  float BC_LAT4;
  float BC_LON4;
  float BC_LAT5;
  float BC_LON5;
} configuration;

// Here is where we define the pins our direction LEDs are attached to.
byte ledDirection[] = {13, 12, 11, 10, 9, 8, 7, 6, 5};

// Here is where we define the pins our distance LEDs are attached to.
byte ledDistance[] = {18, 17, 16, 15, 14};

// Set up the 'SET LOC' pushbutton
//int setLoc = 19;               // set button pin to 19
//int lastButtonState = HIGH;    // the previous buttonState from the input pin
long buttonHoldTime = 0;         // the last time the output pin was toggled
long buttonHoldDelay = 400;      // how long to hold the button down to set BC location
//long buttonDebounce = 10;

// Set up the 'GEO | BC' switch
int modeSwitch = 19;
int lastSwitchState = HIGH;

// Start up settings
int startUp = 0;
int startUpBlinkDelay = 500;
unsigned long startUpBlink = 0;
byte startUpLED = 0;

// Pushbutton setup
int buttonPin = 2;             // the number of the pushbutton pin
int buttonState;               // the current reading from the input pin
int lastButtonState = HIGH;    // the previous reading from the input pin

// the following variables are long's because the time, measured in miliseconds,
// will quickly become a bigger number than can be stored in an int.
long lastDebounceTime = 0;     // the last time the output pin was toggled
long debounceDelay = 50;       // the debounce time; increase if the output flickers
long menuDelay = 2500;
long menuTime;

void gpsdump(TinyGPS &gps);
bool feedgps();

void setup() 
{ 
  byte i;
  
  //Serial.begin(57600);
  nss.begin(9600);
  
  for (i=0; i<9; i++) {
    pinMode(ledDirection[i], OUTPUT);
  }
  
  for (i=0; i<5; i++) {
    pinMode(ledDistance[i], OUTPUT);
  }
  
  // Make input & enable pull-up resistors on switch pins for pushbutton
  pinMode(buttonPin, INPUT);
  digitalWrite(buttonPin, HIGH);
  
  // Make input & enable pull-up resistors on switch pins for pushbutton
  pinMode(modeSwitch, INPUT);
  digitalWrite(modeSwitch, HIGH);
  
  // Load previously saved breadcrumb location from the EEPROM
  EEPROM_readAnything(0, configuration);

}

void loop() 
{  
  bool newdata = false;
  unsigned long start = millis();
  
  // Every 5 seconds we print an update
  while (millis() - start < 100)
  {
    if (feedgps())
      newdata = true;        
  }

  if (newdata)
  {
    gpsdump(gps);
  }
  else {
    if (startUp == 0) {
      if (millis() - startUpBlink > startUpBlinkDelay) {
        clearDirection();
        digitalWrite(ledDirection[startUpLED], HIGH);
        if (startUpLED == 8) {
          startUpLED = 0;
        } else {
          startUpLED++;
        }
        startUpBlink = millis();
      }
    }
    //no new data
  }  
}

void menuCheck()
{
  if (startUp == 0) {
    menu();
  }
  
  // read the state of the switch into a local variable:
  int buttonState = digitalRead(buttonPin);
  int switchState = digitalRead(modeSwitch);

  // check to see if you just pressed the button 
  // (i.e. the input went from LOW to HIGH),  and you've waited 
  // long enough since the last press to ignore any noise:  
  
  // If the switch changed, due to noise or pressing:
  if (buttonState != lastButtonState) {
    // reset the debouncing timer
    lastDebounceTime = millis();
  }
  
  if (buttonState == LOW) {
    menu();
  }
  
  if (switchState != lastSwitchState) {
    menu();
  }
  
  // save the reading.  Next time through the loop,
  // it'll be the lastButtonState and lastSwitchState:
  lastButtonState = buttonState;
  lastSwitchState = switchState;
}
  
void menu() {
  int held = 0;
  menuTime = millis();
  clearDistance();
  clearDirection();
  while((millis() - menuTime) < menuDelay) {
    
    int switchState = digitalRead(modeSwitch);
    int buttonState = digitalRead(buttonPin);
    
    if (buttonState == LOW && lastButtonState == HIGH) {
      buttonHoldTime = millis();
    }
    
    if (buttonState == LOW && lastButtonState == LOW) {
      if ((millis() - buttonHoldTime) > buttonHoldDelay) {
        held = 1;
      }
    }
    
    if (buttonState == HIGH && lastButtonState == LOW) {
      held = 0;
    }
    
    if (switchState != lastSwitchState || startUp == 0) { 
      clearDistance();
      if (switchState == LOW) {
        targetFlat = geoLAT[currentWP];
        targetFlon = geoLON[currentWP];
        tripDistance = (double)calc_dist(lastFlat, lastFlon, targetFlat, targetFlon);
      } else {
        if (currentBC == 0) {
          targetFlat = configuration.BC_LAT1;
          targetFlon = configuration.BC_LON1;
        }
        if (currentBC == 1) {
          targetFlat = configuration.BC_LAT2;
          targetFlon = configuration.BC_LON2;
        }
        if (currentBC == 2) {
          targetFlat = configuration.BC_LAT3;
          targetFlon = configuration.BC_LON3;
        }
        if (currentBC == 3) {
          targetFlat = configuration.BC_LAT4;
          targetFlon = configuration.BC_LON4;
        }
        if (currentBC == 4) {
          targetFlat = configuration.BC_LAT5;
          targetFlon = configuration.BC_LON5;
        }
        tripDistance = (double)calc_dist(lastFlat, lastFlon, targetFlat, targetFlon);
      }
      startUp = 1;
    }
    
    if (switchState == LOW) { //GEO
      
      digitalWrite(ledDistance[currentWP], HIGH);
      
      if (buttonState == HIGH && lastButtonState == LOW) {
        if ((millis() - buttonHoldTime) > debounceDelay) {
          if (currentWP == 4) {
            currentWP = 0;
          } else {
            currentWP++;
          }
          clearDistance();
          digitalWrite(ledDistance[currentWP], HIGH);
          targetFlat = geoLAT[currentWP];
          targetFlon = geoLON[currentWP];
          tripDistance = (double)calc_dist(lastFlat, lastFlon, targetFlat, targetFlon);
          menuTime = millis();
          held = 0;
        }
      }
    } else {
      
      digitalWrite(ledDistance[currentBC], HIGH);
      
      if (buttonState == HIGH && lastButtonState == LOW) {
        if (((millis() - buttonHoldTime) > debounceDelay) && held != 1) {
          if (currentBC == 4) {
            currentBC = 0;
          } else {
            currentBC++;
          }
          clearDistance();
          digitalWrite(ledDistance[currentBC], HIGH);
          menuTime = millis();
        }  
      }
      
      if (held == 1 && buttonState == LOW) {
        clearDirection();
        while(buttonState == LOW){
          
          if ((millis() - buttonHoldTime) > (buttonHoldDelay)){
            digitalWrite(ledDirection[0], HIGH);
          }
          
          if ((millis() - buttonHoldTime) > (buttonHoldDelay*2)){
            digitalWrite(ledDirection[1], HIGH);
          }
          
          if ((millis() - buttonHoldTime) > (buttonHoldDelay*3)){
            digitalWrite(ledDirection[2], HIGH);
          }
          
          if ((millis() - buttonHoldTime) > (buttonHoldDelay*4)){
            digitalWrite(ledDirection[3], HIGH);
          }
          
          if ((millis() - buttonHoldTime) > (buttonHoldDelay*5)){
            digitalWrite(ledDirection[4], HIGH);
          }
          
          if ((millis() - buttonHoldTime) > (buttonHoldDelay*6)){
            digitalWrite(ledDirection[5], HIGH);
          }
          
          if ((millis() - buttonHoldTime) > (buttonHoldDelay*7)){
            digitalWrite(ledDirection[6], HIGH);
          }
          
          if ((millis() - buttonHoldTime) > (buttonHoldDelay*8)){
            digitalWrite(ledDirection[7], HIGH);
          }
          
          if ((millis() - buttonHoldTime) > (buttonHoldDelay*9)){
            digitalWrite(ledDirection[8], HIGH);
            
            if (currentBC == 0) {
              configuration.BC_LAT1 = lastFlat;
              configuration.BC_LON1 = lastFlon;
            }
            if (currentBC == 1) {
              configuration.BC_LAT2 = lastFlat;
              configuration.BC_LON2 = lastFlon;
            }
            if (currentBC == 2) {
              configuration.BC_LAT3 = lastFlat;
              configuration.BC_LON3 = lastFlon;
            }
            if (currentBC == 3) {
              configuration.BC_LAT4 = lastFlat;
              configuration.BC_LON4 = lastFlon;
            }
            if (currentBC == 4) {
              configuration.BC_LAT5 = lastFlat;
              configuration.BC_LON5 = lastFlon;
            }
            
            EEPROM_writeAnything(0, configuration);
            clearDirection();
            clearDistance();
            
            delay(500);
            digitalWrite(ledDistance[currentBC], HIGH);
            delay(500);
            digitalWrite(ledDistance[currentBC], LOW);
            delay(500);
            digitalWrite(ledDistance[currentBC], HIGH);
            delay(500);
            digitalWrite(ledDistance[currentBC], LOW);
            delay(500);
            digitalWrite(ledDistance[currentBC], HIGH);
            
            buttonHoldTime = millis();
            menuTime = millis();
            held = 0;
           
          }
          buttonState = digitalRead(buttonPin);
        }
      }
      
      if (currentBC == 0) {
        targetFlat = configuration.BC_LAT1;
        targetFlon = configuration.BC_LON1;
      }
      if (currentBC == 1) {
        targetFlat = configuration.BC_LAT2;
        targetFlon = configuration.BC_LON2;
      }
      if (currentBC == 2) {
        targetFlat = configuration.BC_LAT3;
        targetFlon = configuration.BC_LON3;
      }
      if (currentBC == 3) {
        targetFlat = configuration.BC_LAT4;
        targetFlon = configuration.BC_LON4;
      }
      if (currentBC == 4) {
        targetFlat = configuration.BC_LAT5;
        targetFlon = configuration.BC_LON5;
      }
      tripDistance = (double)calc_dist(lastFlat, lastFlon, targetFlat, targetFlon);  
    }
  lastButtonState = buttonState;
  lastSwitchState = switchState;
  }
}

void gpsdump(TinyGPS &gps)
{
  float flat, flon;
  unsigned long age;

  feedgps(); // If we don't feed the gps during this long routine, we may drop characters and get checksum errors

  gps.f_get_position(&flat, &flon, &age);

  if (age > 30000)
  {
    //warning!!!
  }
  
  lastFlat = flat;
  lastFlon = flon;
  menuCheck();
  feedgps();

  if ((calc_bearing(flat, flon, targetFlat, targetFlon) - gps.f_course()) > 0) {
    headingDirection(calc_bearing(flat, flon, targetFlat, targetFlon)-gps.f_course());
  }
  else {
    headingDirection(calc_bearing(flat, flon, targetFlat, targetFlon)-gps.f_course()+360);
  }
  menuCheck();
  feedgps();
  
  headingDistance((double)calc_dist(flat, flon, targetFlat, targetFlon));
  
  menuCheck();
}

bool feedgps()
{
  while (nss.available())
  {
    if (gps.encode(nss.read()))
      return true;
  }
  return false;
}

void headingDirection(float heading) 
{
  //Here we will use the ledDirection array to turn on the correct LED
  //Remember: arrays are zero indexed
  if ((heading > 348.75)||(heading < 11.25)) {
    //Serial.print("  N");
    clearDirection();
    digitalWrite(ledDirection[4], HIGH);
  }
  
  if ((heading >= 11.25)&&(heading < 33.75)) {
    //Serial.print("NNE");
    clearDirection();
    digitalWrite(ledDirection[5], HIGH);
  }  
  
  if ((heading >= 33.75)&&(heading < 56.25)) {
    //Serial.print(" NE");
    clearDirection();
    digitalWrite(ledDirection[6], HIGH);
  }
  
  if ((heading >= 56.25)&&(heading < 78.75)) {
    //Serial.print("ENE");
    clearDirection();
    digitalWrite(ledDirection[7], HIGH);
  }
  
  if ((heading >= 78.75)&&(heading < 101.25)) {
    //Serial.print("  E");
    clearDirection();
    digitalWrite(ledDirection[8], HIGH);
  }
  
  if ((heading >= 101.25)&&(heading < 123.75)) {
    //Serial.print("ESE");
    clearDirection();
    digitalWrite(ledDirection[8], HIGH);
  }
  
  if ((heading >= 123.75)&&(heading < 146.25)) {
    //Serial.print(" SE");
    clearDirection();
    digitalWrite(ledDirection[8], HIGH);
  }
  
  if ((heading >= 146.25)&&(heading < 168.75)) {
    //Serial.print("SSE");
    clearDirection();
    digitalWrite(ledDirection[8], HIGH);
  }
  
  if ((heading >= 168.75)&&(heading < 191.25)) {
    //Serial.print("  S");
    clearDirection();
    digitalWrite(ledDirection[8], HIGH);
    digitalWrite(ledDirection[0], HIGH);
  }
  
  if ((heading >= 191.25)&&(heading < 213.75)) {
    //Serial.print("SSW");
    clearDirection();
    digitalWrite(ledDirection[0], HIGH);
  }
  
  if ((heading >= 213.75)&&(heading < 236.25)) {
    //Serial.print(" SW");
    clearDirection();
    digitalWrite(ledDirection[0], HIGH);
  }
  
  if ((heading >= 236.25)&&(heading < 258.75)) {
    //Serial.print("WSW");
    clearDirection();
    digitalWrite(ledDirection[0], HIGH);
  }
  
  if ((heading >= 258.75)&&(heading < 281.25)) {
    //Serial.print("  W");
    clearDirection();
    digitalWrite(ledDirection[0], HIGH);
  }
  
  if ((heading >= 281.25)&&(heading < 303.75)) {
    //Serial.print("WNW");
    clearDirection();
    digitalWrite(ledDirection[1], HIGH);
  }
  
  if ((heading >= 303.75)&&(heading < 326.25)) {
    //Serial.print(" NW");
    clearDirection();
    digitalWrite(ledDirection[2], HIGH);
  }
  
  if ((heading >= 326.25)&&(heading < 348.75)) {
    //Serial.print("NWN");
    clearDirection();
    digitalWrite(ledDirection[3], HIGH);
  }
}

int calc_bearing(float flat1, float flon1, float flat2, float flon2)
{
  float calc;
  float bear_calc;

  float x = 69.1 * (flat2 - flat1); 
  float y = 69.1 * (flon2 - flon1) * cos(flat1/57.3);

  calc=atan2(y,x);

  bear_calc= degrees(calc);

  if(bear_calc<=1){
    bear_calc=360+bear_calc; 
  }
  return bear_calc;
}  

void headingDistance(float fDist)
{
  //Here we will use the ledDistance array to turn on the correct LED
  //Remember: arrays are zero indexed (the start LED is actually ledDistance[0]).
  float tripSegment = tripDistance/5; // Take the total trip distance, and break it up evenly for each LED.
  
  if (fDist >= (tripSegment*4)) {
    clearDistance();
    digitalWrite(ledDistance[0], HIGH);
  }
  if ((fDist >= (tripSegment*3))&&(fDist < (tripSegment*4))) {
    clearDistance();
    digitalWrite(ledDistance[1], HIGH);
  }
  if ((fDist >= (tripSegment*2))&&(fDist < (tripSegment*3))) {
    clearDistance();
    digitalWrite(ledDistance[2], HIGH);
  }
  if ((fDist >= tripSegment)&&(fDist < (tripSegment*2))) {
    clearDistance();
    digitalWrite(ledDistance[3], HIGH);
  }
  if ((fDist >= 5)&&(fDist < tripSegment)) {
    clearDistance();
    digitalWrite(ledDistance[4], HIGH);
  }

  if ((fDist < 5) && (fDist >= 4)) { // Flip on ALL distance LEDs if we are within 5 meters of the target.
    allDistance();
  }
  if ((fDist < 4) && (fDist >= 3)) { // Flip on FOUR distance LEDs if we are within 4 meters of the target.
    clearDistance();
    digitalWrite(ledDistance[1], HIGH);
    digitalWrite(ledDistance[2], HIGH);
    digitalWrite(ledDistance[3], HIGH);
    digitalWrite(ledDistance[4], HIGH);
  }
  if ((fDist < 3) && (fDist >= 2)) { // Flip on THREE distance LEDs if we are within 3 meters of the target.
    clearDistance();
    digitalWrite(ledDistance[2], HIGH);
    digitalWrite(ledDistance[3], HIGH);
    digitalWrite(ledDistance[4], HIGH);
  }
  if ((fDist < 2) && (fDist >= 1)) { // Flip on TWO distance LEDs if we are within 2 meters of the target.
    clearDistance();
    digitalWrite(ledDistance[3], HIGH);
    digitalWrite(ledDistance[4], HIGH);
  }
  if (fDist < 1) { // Flip on ONE distance LEDs if we are within 1 meter of the target.
    clearDistance();
    digitalWrite(ledDistance[4], HIGH);
  }
}

unsigned long calc_dist(float flat1, float flon1, float flat2, float flon2)
{
  float dist_calc=0;
  float dist_calc2=0;
  float diflat=0;
  float diflon=0;

  diflat=radians(flat2-flat1);
  flat1=radians(flat1);
  flat2=radians(flat2);
  diflon=radians((flon2)-(flon1));

  dist_calc = (sin(diflat/2.0)*sin(diflat/2.0));
  dist_calc2= cos(flat1);
  dist_calc2*=cos(flat2);
  dist_calc2*=sin(diflon/2.0);
  dist_calc2*=sin(diflon/2.0);
  dist_calc +=dist_calc2;

  dist_calc=(2*atan2(sqrt(dist_calc),sqrt(1.0-dist_calc)));

  dist_calc*=6371000.0; //Converting to meters
  return dist_calc;
}

void clearDirection() //turn off all direction LEDs
{
  byte i;
  for (i=0; i<9; i++) {
    digitalWrite(ledDirection[i], LOW);
  }
}

void clearDistance() //turn off all distance LEDs
{
  byte i;
  for (i=0; i<5; i++) {
    digitalWrite(ledDistance[i], LOW);
  }
}

void allDistance() //turn on all distance LEDs
{
  byte i;
  for (i=0; i<5; i++) {
    digitalWrite(ledDistance[i], HIGH);
  }
}




