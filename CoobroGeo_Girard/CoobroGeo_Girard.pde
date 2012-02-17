/*******************************************************************************
* Title: Coobro Geo
* Version: 1.00
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
* 1.10      Mods by gerardf:
*           Serial commando interpreter to read/write breadcrumbs and waypoints
*           commands: * = enter command mode, r = read waypoints, 
*           commands: * = w=write waypoints,  q=quit command mode
*           structure to write: w b17 +00.000000 +000.000000 *
*           Breadcrumbs set to 32, distance LED indicate binay number
*******************************************************************************/
#include <NewSoftSerial.h>
#include <TinyGPS.h>
#include <EEPROM.h>
#include "EEPROMAnything.h" // Required to save floats into EEPROM
#include <math.h>

TinyGPS gps;
NewSoftSerial nss(4, 3);

#define  HIGHRES_DIST              10    // high resolution distance in meter (where we turn on all distance led's)
#define  MAXBREADCRUMBS            32    // max 32
#define  MAXWAYPOINTS              32    // max 32

//--------------------------------------------------|
//                    WAYPOINTS                     |
//--------------------------------------------------|
//Please enter the latitude and longitude of your   |
//desired destinations:                              |
  #define GEO_LAT1                51.472915  // Home
  #define GEO_LON1                 7.760877
  #define GEO_LAT2                51.472200  // Aldi
  #define GEO_LON2                 7.764321
  #define GEO_LAT3                51.539725  // NAK Massen
  #define GEO_LON3                 7.645819
  #define GEO_LAT4                51.471913  // Bahnuebergang
  #define GEO_LON4                 7.768178
  #define GEO_LAT5                51.448924  // Kaufland
  #define GEO_LON5                 7.757672
  #define GEO_LAT6                51.477678  // NAK Froendenberg
  #define GEO_LON6                 7.75767
  #define GEO_LAT7                51.471512  // Bahnhof
  #define GEO_LON7                 7.762375
  #define GEO_LAT8                51.472195  // Steuerberater
  #define GEO_LON8                 7.766770

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

float geoLAT[MAXWAYPOINTS] = {GEO_LAT1, GEO_LAT2, GEO_LAT3, GEO_LAT4, GEO_LAT5, GEO_LAT6, GEO_LAT7, GEO_LAT8};
float geoLON[MAXWAYPOINTS] = {GEO_LON1, GEO_LON2, GEO_LON3, GEO_LON4, GEO_LON5, GEO_LON6, GEO_LON7, GEO_LON8};

// Waypoints location
struct wploc
{
  float GEO_LAT;
  float GEO_LON;
} waypoints[MAXWAYPOINTS];

// Breadcrumbs location
struct bcloc
{
  float BC_LAT;
  float BC_LON;
} breadcrumbs[MAXBREADCRUMBS];



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
  
  Serial.begin(19200);   
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
  for (int i = 0; i < MAXBREADCRUMBS ; i++){
    EEPROM_readAnything(i*8, breadcrumbs[i]);
  }
  // Load previously saved waypoints location from the EEPROM
  for (int i = 0; i < MAXWAYPOINTS ; i++){
    EEPROM_readAnything((MAXBREADCRUMBS*8)+(i*8), waypoints[i]);
  }
  // Transfer the waypoints into work array (only if work array field is empty
  for (int i = 0; i < MAXWAYPOINTS ; i++){
    if (geoLAT[i] && geoLON[i] == 0) {
      geoLAT[i] = waypoints[i].GEO_LAT;
      geoLON[i] = waypoints[i].GEO_LON;     
    } 
  }
}

void loop() 
{  
  bool newdata = false;
  unsigned long start = millis();
  
  // check if serial data available
  if (Serial.available() > 0) {
    if (Serial.read() == '*')
      com_menu();  // handle serial communication menu
    }

  // Every 5 seconds we print an update
  while (millis() - start < 100) {
    if (feedgps())
      newdata = true;        
  }

  if (newdata) {
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


// handle serial communication menu
void com_menu()
{
  unsigned long start = millis();    // for menu timeout
  char command ;
  char input[32];   // data buffer
  char current;     // actual character
  int incount;      // buffer counter
  bool lineComplete;  
  char wpt;         // waypoint typ (G or B)
  int wpn;          // waypoint number
  int wpc;          // for conversion
  float wplat;      // waypoint lat
  float wplon;      // waypoint lon
  int indexhelper;  // debug

  while (millis() - start < 60000) { // max 60 sec inactivity
    if (Serial.available() > 0) {
      start = millis();              // reset timeout counter  
      command = Serial.read() ;      // get the input char 
      switch (command) {
        case 'q':                    // quit command?
          return ;
        case 'r':                    // read command
          Serial.println("Entering read command:"); 
          for (int i = 0; i < MAXBREADCRUMBS ; i++){
            EEPROM_readAnything(i*8, breadcrumbs[i]);
          }  
          for (int i= 0; i< MAXWAYPOINTS; i++){    // display waypoint eeprom storage
            if (waypoints[i].GEO_LAT > 0 || waypoints[i].GEO_LON > 0 || waypoints[i].GEO_LAT < 0 || waypoints[i].GEO_LON < 0) {
            // this wilde if-condition is necessary as the != comparator did not work for unknown reasons
              Serial.print("eep_LAT");
              Serial.print(i, DEC);
              Serial.print(": ");
              Serial.println(waypoints[i].GEO_LAT, DEC); 
              Serial.print("eep_LON");
              Serial.print(i, DEC);
              Serial.print(": ");
              Serial.println(waypoints[i].GEO_LON, DEC);            
            }
          }

          for (int i= 0; i< MAXWAYPOINTS; i++){    // display waypoint flash storage
            if (geoLAT[i] > 0 || geoLON[i] > 0 || geoLAT[i] < 0 || geoLON[i] < 0) {
            // this wilde if-condition is necessary as the != comparator did not work for unknown reasons
              Serial.print("flash_LAT");
              Serial.print(i, DEC);
              Serial.print(": ");
              Serial.println(geoLAT[i], DEC); 
              Serial.print("flash_LON");
              Serial.print(i, DEC);
              Serial.print(": ");
              Serial.println(geoLON[i], DEC);            
            }
          }            
          
          for (int i= 0; i< MAXBREADCRUMBS; i++){    // display breadcrumbs eeprom storage
            if (breadcrumbs[i].BC_LAT > 0 || breadcrumbs[i].BC_LON > 0 || breadcrumbs[i].BC_LAT < 0 || breadcrumbs[i].BC_LON < 0) {
            // this wilde if-condition is necessary as the != comparator did not work for unknown reasons
              Serial.print("BC_LAT");
              Serial.print(i, DEC);
              Serial.print(": ");
              Serial.println(breadcrumbs[i].BC_LAT, DEC); 
              Serial.print("BC_LON");
              Serial.print(i, DEC);
              Serial.print(": ");
              Serial.println(breadcrumbs[i].BC_LON, DEC);            
            }
          }
          Serial.println("  ");
          break;
          

        case 'w':                    // write command  Expected format: w gxx syy.yyyyyy szzz.zzzzzz *
          lineComplete = false;
          incount = 0;             // reset counter
          while (millis() - start < 60000 & (incount < 32) & (!lineComplete)) { // max 60 sec inactivity
            if (Serial.available() > 0) {
              start = millis();              // reset timeout counter            
              current = Serial.read(); 
              if (current != 42) {     // any char except *(=end of line) 
                input[incount] = current;   // fill buffer
                incount++;
              }
              else {                   // we have *(=end of line)
                input[incount] = '\0'; // close the string
                lineComplete = true;
              }
            }
            if (lineComplete) {        // content should be:  bgxx syy.yyyyyy szzz.zzzzzz 
              Serial.println("Entering write command:"); 
              wpt = input[1];            // either g for Geo_data or b for Bread_crumb
              Serial.print("wpt: ");
              Serial.write(wpt);

              wpn = (input[2]-48)*10 + (input[3]-48);  // waypoint number 00-31
              wplat = (input[9]-48)*0.1 + (input[10]-48)*0.01 + (input[11]-48)*0.001 + (input[12]-48)*0.0001 + (input[13]-48)*0.00001 + (input[14]-48)*0.000001;
              wplat = wplat + (input[6]-48)*10 + (input[7]-48);
              if (input[5] == '-')
                wplat = wplat * -1;

              wplon = (input[21]-48)*0.1 + (input[22]-48)*0.01 + (input[23]-48)*0.001 + (input[24]-48)*0.0001 + (input[25]-48)*0.00001 + (input[26]-48)*0.000001;
              wplon = wplon + (input[17]-48)*100 + (input[18]-48)*10 + (input[19]-48);
              if (input[16] == '-')
                wplon = wplon * -1;

              Serial.print("  wpn: ");
              Serial.print(wpn, DEC);
              Serial.print("  wplat: ");
              Serial.print(wplat, DEC);
              Serial.print("  wplon: ");
              Serial.println(wplon, DEC);
              if (wpt == 'b') {        // target is breadcrumb, not waypoint
                breadcrumbs[wpn].BC_LAT = wplat;
                breadcrumbs[wpn].BC_LON = wplon;
                EEPROM_writeAnything(wpn*8, breadcrumbs[wpn]);
              }
              else {                  // target is waypoint, not breadcrumb
                waypoints[wpn].GEO_LAT = wplat;
                waypoints[wpn].GEO_LON = wplon;
                EEPROM_writeAnything((MAXBREADCRUMBS*8)+(wpn*8), waypoints[wpn]);
              }
            }
          }
        Serial.println("  ");
        break;    // from case: w
      }
    }
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
        targetFlat = breadcrumbs[currentBC].BC_LAT ;
        targetFlon = breadcrumbs[currentBC].BC_LON ;
        tripDistance = (double)calc_dist(lastFlat, lastFlon, targetFlat, targetFlon);
      }
      startUp = 1;
    }
    
    if (switchState == LOW) { //GEO
      
      showbctr(currentWP);
      
      if (buttonState == HIGH && lastButtonState == LOW) {
        if ((millis() - buttonHoldTime) > debounceDelay) {
          if (currentWP == MAXWAYPOINTS-1) {
            currentWP = 0;
          } else {
            currentWP++;
          }
          clearDistance();
          showbctr(currentWP); 

          targetFlat = geoLAT[currentWP];
          targetFlon = geoLON[currentWP];
          tripDistance = (double)calc_dist(lastFlat, lastFlon, targetFlat, targetFlon);
          menuTime = millis();
          held = 0;
        }
      }
    } else {
      
      showbctr(currentBC);
      
      if (buttonState == HIGH && lastButtonState == LOW) {
        if (((millis() - buttonHoldTime) > debounceDelay) && held != 1) {
          if (currentBC == MAXBREADCRUMBS-1) {
            currentBC = 0;
          } else {
            currentBC++;
          }
          clearDistance();
          showbctr(currentBC);
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
            
            breadcrumbs[currentBC].BC_LAT = lastFlat ;
            breadcrumbs[currentBC].BC_LON = lastFlon ;

            for (int i = 0; i < MAXBREADCRUMBS ; i++){
              EEPROM_writeAnything(i*8, breadcrumbs[i]);
            }


            clearDirection();
            clearDistance();
            
            delay(500);
            showbctr(currentBC);
            delay(500);
            clearDistance();
            delay(500);
            showbctr(currentBC);
            delay(500);
            clearDistance();
            delay(500);
            showbctr(currentBC);
            
            buttonHoldTime = millis();
            menuTime = millis();
            held = 0;
           
          }
          buttonState = digitalRead(buttonPin);
        }
      }
      
      targetFlat = breadcrumbs[currentBC].BC_LAT ;
      targetFlon = breadcrumbs[currentBC].BC_LON ;
      tripDistance = (double)calc_dist(lastFlat, lastFlon, targetFlat, targetFlon);  
    }
  lastButtonState = buttonState;
  lastSwitchState = switchState;
  }
}


// show binary counter on distance led, input currentBC
// replace;     digitalWrite(ledDistance[currentBC], HIGH);
void showbctr(int cbc){
  int i = 1;
  
  for (int j = 0; j < 5; j++){     // check 5 led's
    if (cbc & i) {                // bit set?
      digitalWrite(ledDistance[4-j], HIGH);  // turn on LED
    }
  i = i << 1;                      // next bit to check
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
    // Serial.print("  N");    
    // Serial.println(heading);
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
  if ((fDist >= HIGHRES_DIST)&&(fDist < tripSegment)) {
    clearDistance();
    digitalWrite(ledDistance[4], HIGH);
  }

  if ((fDist < HIGHRES_DIST) && (fDist >= HIGHRES_DIST*0.8)) { // Flip on ALL distance LEDs if we are within HIGHRES_DIST of the target.
    allDistance();
  }
  if ((fDist < HIGHRES_DIST*0.8) && (fDist >= HIGHRES_DIST*0.6)) { // Flip on FOUR ...if we are within 80% of HIGHRES_DIST of the target.
    clearDistance();
    digitalWrite(ledDistance[1], HIGH);
    digitalWrite(ledDistance[2], HIGH);
    digitalWrite(ledDistance[3], HIGH);
    digitalWrite(ledDistance[4], HIGH);
  }
  if ((fDist < HIGHRES_DIST*0.6) && (fDist >= HIGHRES_DIST*0.4)) { // Flip on FOUR ...if we are within 60% of HIGHRES_DIST of the target.
    clearDistance();
    digitalWrite(ledDistance[2], HIGH);
    digitalWrite(ledDistance[3], HIGH);
    digitalWrite(ledDistance[4], HIGH);
  }
  if ((fDist < HIGHRES_DIST*0.4) && (fDist >= HIGHRES_DIST*0.2)) { // Flip on FOUR ...if we are within 40% of HIGHRES_DIST of the target.
    clearDistance();
    digitalWrite(ledDistance[3], HIGH);
    digitalWrite(ledDistance[4], HIGH);
  }
  if (fDist < HIGHRES_DIST*0.2) { // Flip on FOUR ...if we are within 20% of HIGHRES_DIST of the target.
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



