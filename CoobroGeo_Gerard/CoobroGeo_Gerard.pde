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
* 1.30      added activity logging: every navigation start we log start LAT/LON
*           target LAT/LON and start Date / Time in revolving buffer, 10 entries
*           added command * d = dump log to serial out
*           added software version display (binary) on dist_LED at boot time
*           change in preset target functionality: whenever a wp or bc target 
*           from the eeprom is picked by the user, this target is also saved in
*           eeprom wp or bc position 00, therefore new default in case the GPS
*           is turned of and on again during navigation. This is to not have to
*           pick again the last selection. 
* 1.31      target selection from wp / bc is now possible before we have a gps lock
*           corrected bug in read command
*           implemented new command: move:  m gxx byy *  / any g/b to any g/b
* 1.32      setup logic changed: eeprom waypoint preceed PROGMEM geoData
*           r-command will print waypoint AND flash data even if they are identical
* 1.33      r-command does not show flash storage anymore (going out of ram?)
*******************************************************************************/
#include <NewSoftSerial.h>
#include <TinyGPS.h>
#include <EEPROM.h>
#include "EEPROMAnything.h" // Required to save floats into EEPROM
#include <math.h>

TinyGPS gps;
NewSoftSerial nss(4, 3);

#define  SOFTWARE_VERSION          03    // FOR DISPLAY AT SWITCH-ON, Max value = 31
#define  SOFTWARE_PRINT_VERSION    1.33  // FOR Serial.print at SWITCH-ON, Max 4 char
#define  HIGHRES_DIST              10    // high resolution distance in meter (where we turn on all distance led's)
#define  MAXBREADCRUMBS            32    // max 32
#define  MAXWAYPOINTS              32    // max 32
#define  MAXLOGS                   10    // 10 log table entries

//--------------------------------------------------|
//                    WAYPOINTS                     |
//--------------------------------------------------|
//Please enter the latitude and longitude of your   |
//desired destinations:                              |
  
  #define GEO_LAT0                 0         // keep zero
  #define GEO_LON0                 0  


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

float geoLAT[MAXWAYPOINTS] = {GEO_LAT0};
float geoLON[MAXWAYPOINTS] = {GEO_LON0};

// Waypoints location:    eeprom offset 256
struct wploc
{
  float GEO_LAT;
  float GEO_LON;
} waypoints[MAXWAYPOINTS];

// Breadcrumbs location:  eeprom offset 0
struct bcloc
{
  float BC_LAT;
  float BC_LON;
} breadcrumbs[MAXBREADCRUMBS];

// Logging location:  eeprom offset 512 FOR POINTER,  514 FOR STRUCT logloc
  int indexptr;  // pointer to datalog
struct logloc
{
  float LOG_STARTLAT;
  float LOG_STARTLON;
  unsigned long LOG_DATE;
  unsigned long LOG_TIME;
  float LOG_TARGETLAT;
  float LOG_TARGETLON;
} datalog[MAXLOGS];    // MAXLOGS * 24 bytes

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
  // Transfer the waypoints into work array (only if waypoint field is not empty
  for (int i = 0; i < MAXWAYPOINTS ; i++){
//    if (waypoints[i].GEO_LAT + waypoints[i].GEO_LON == 0) {
      geoLAT[i] = waypoints[i].GEO_LAT;
      geoLON[i] = waypoints[i].GEO_LON;     
//    } 
  }
  for (int i=0; i<5; i++){
    clearDistance();
    delay(300);
    showbctr(SOFTWARE_VERSION);        // display software version at switch-on
    delay(300);
  }
  EEPROM_readAnything(512, indexptr); // get pointer for logging
  if (indexptr >= MAXLOGS)
    indexptr = 0;
  Serial.print("Software version: ");
  Serial.println(SOFTWARE_PRINT_VERSION,2);
  Serial.print(" ");
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
    if (!digitalRead(buttonPin))
      setTarget();     // to get a chance to set target before we have a lock 
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
  char wptmt;       // waypoint typ (G or B) move to
  int wpn;          // waypoint number
  int wpnmt;        // waypoint number move to
  int wpc;          // for conversion
  float wplat;      // waypoint lat
  float wplon;      // waypoint lon

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

//          for (int i= 0; i< MAXWAYPOINTS; i++){    // display waypoint flash storage
//            if (geoLAT[i] > 0 || geoLON[i] > 0 || geoLAT[i] < 0 || geoLON[i] < 0) {
            // this wilde if-condition is necessary as the != comparator did not work for unknown reasons
//              if (geoLAT[i] - waypoints[i].GEO_LAT +  geoLON[i] - waypoints[i].GEO_LON != 0){ // geoLAT/LON != waypointsLAT/LON
//                Serial.print("flash_LAT");
//                Serial.print(i, DEC);
//                Serial.print(": ");
//                Serial.println(geoLAT[i], DEC); 
//                Serial.print("flash_LON");
//                Serial.print(i, DEC);
//                Serial.print(": ");
//                Serial.println(geoLON[i], DEC);            
//              }
//            }
//          }            
          
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

        case 'd':                    // dump log command
          Serial.print("Printing log (last log#) :");
          if (indexptr == 0)
            Serial.println(MAXLOGS-1, DEC);
          else
            Serial.println(indexptr-1, DEC);
          for (int i = 0; i < MAXLOGS ; i++){
            EEPROM_readAnything((514)+i*24, datalog[i]);  // get one log
            Serial.print("Log #: ");
            Serial.println(i, DEC);
            Serial.print("Start  LAT: ");
            Serial.print(datalog[i].LOG_STARTLAT, DEC);
            Serial.print("   LON: ");
            Serial.println(datalog[i].LOG_STARTLON, DEC);
            Serial.print("Target LAT: ");
            Serial.print(datalog[i].LOG_TARGETLAT, DEC);
            Serial.print("   LON: ");
            Serial.println(datalog[i].LOG_TARGETLON, DEC);
            Serial.print("Date (ddMMYY): ");
            Serial.print(datalog[i].LOG_DATE, DEC);
            Serial.print("   Time (hhmmss): ");
            Serial.println(datalog[i].LOG_TIME, DEC);
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
        

        case 'm':                    // move command  Expected format: m gxx gyy *
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
            if (lineComplete) {        // content should be:  bgxx gyy  
              Serial.println("Entering move command:"); 
              wpt = input[1];            // either g for Geo_data or b for Bread_crumb
              Serial.print("Moving wpt: ");
              Serial.write(wpt);
              wpn = (input[2]-48)*10 + (input[3]-48);  // waypoint number 00-31
              Serial.print(wpn, DEC);
              wptmt = input[5];            // either g for Geo_data or b for Bread_crumb
              Serial.print("  to wpt: ");
              Serial.write(wptmt);
              wpnmt = (input[6]-48)*10 + (input[7]-48);  // waypoint number 00-31
              Serial.println(wpnmt, DEC);
              if (wpt == 'b' && wptmt == 'b') {         // target and source is breadcrumb
                breadcrumbs[wpnmt].BC_LAT = breadcrumbs[wpn].BC_LAT;
                breadcrumbs[wpnmt].BC_LON = breadcrumbs[wpn].BC_LON;
                EEPROM_writeAnything(wpnmt*8, breadcrumbs[wpnmt]);
              }  
              if (wpt == 'g' && wptmt == 'b') {         // target is breadcrumb, source is waypoint
                breadcrumbs[wpnmt].BC_LAT = waypoints[wpn].GEO_LAT;
                breadcrumbs[wpnmt].BC_LON = waypoints[wpn].GEO_LON;
                EEPROM_writeAnything(wpnmt*8, breadcrumbs[wpnmt]);
              }
              if (wpt == 'b' && wptmt == 'g') {         // target is waypoint and source is breadcrumb
                waypoints[wpnmt].GEO_LAT = breadcrumbs[wpn].BC_LAT;
                waypoints[wpnmt].GEO_LON = breadcrumbs[wpn].BC_LON;
                EEPROM_writeAnything((MAXBREADCRUMBS*8)+(wpnmt*8), waypoints[wpnmt]);
              }
              if (wpt == 'g' && wptmt == 'g') {         // target and source is waypoint
                waypoints[wpnmt].GEO_LAT = waypoints[wpn].GEO_LAT;
                waypoints[wpnmt].GEO_LON = waypoints[wpn].GEO_LON;
                EEPROM_writeAnything((MAXBREADCRUMBS*8)+(wpnmt*8), waypoints[wpnmt]);
              }
            }
          }
        Serial.println("  ");
        break;    // from case: m
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
      if (switchState == LOW) {      //GEO
        targetFlat = geoLAT[currentWP];
        targetFlon = geoLON[currentWP];
        tripDistance = (double)calc_dist(lastFlat, lastFlon, targetFlat, targetFlon);
      } else {                      // switch state = Breadcrumbs
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
    } else {              // switch state = Breadcrumbs
      
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
            
            for (int i=0; i<3; i++){
              delay(500);
              clearDistance();
              delay(500);
              showbctr(currentBC);
            }
            
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
  if (digitalRead(modeSwitch) == LOW) {      //GEO
    waypoints[0].GEO_LAT = targetFlat ;
    waypoints[0].GEO_LON = targetFlon ;
    EEPROM_writeAnything((MAXBREADCRUMBS*8), waypoints[0]);  // save target to wp00
  } else {                      // switch state = Breadcrumbs
    breadcrumbs[0].BC_LAT = targetFlat ;
    breadcrumbs[0].BC_LON = targetFlon ;
    EEPROM_writeAnything(0, breadcrumbs[0]);                // save target to bc00
  }
// log at navigation start if we have a fix
  long flat, flon;
  unsigned long fix_age;
  gps.get_position(&flat, &flon, &fix_age);
  if (fix_age == TinyGPS::GPS_INVALID_AGE)
     Serial.println("No fix detected");
  else if (fix_age > 5000)
    Serial.println("Warning: possible stale data!");
  else {
//    Serial.println("Data is current.");
    datalog[indexptr].LOG_STARTLAT = lastFlat;
    datalog[indexptr].LOG_STARTLON = lastFlon;
    datalog[indexptr].LOG_TARGETLAT = targetFlat;
    datalog[indexptr].LOG_TARGETLON = targetFlon;
    gps.get_datetime(&datalog[indexptr].LOG_DATE, &datalog[indexptr].LOG_TIME);
    datalog[indexptr].LOG_TIME = datalog[indexptr].LOG_TIME / 100;     // remove hundreds of sec
    EEPROM_writeAnything(514 + (indexptr*24), datalog[indexptr]);      // save log
    indexptr++;
    if (indexptr >= MAXLOGS)
      indexptr = 0;
    EEPROM_writeAnything(512, indexptr);               // and save pointer
   }
}


void setTarget() {

  menuTime = millis();

  while((millis() - menuTime) < menuDelay) {
    
    int switchState = digitalRead(modeSwitch);
    int buttonState = digitalRead(buttonPin);

    clearDistance();

    if (buttonState == LOW && lastButtonState == HIGH) 
      buttonHoldTime = millis();
      
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

          menuTime = millis();
        }
      }
    } else {              // switch state = Breadcrumbs
      
      showbctr(currentBC);
    
      if (buttonState == HIGH && lastButtonState == LOW) {
        if (((millis() - buttonHoldTime) > debounceDelay)) {
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
//  Serial.println(fDist, DEC);  // debug
  
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



