/*****************************************************************************

    File Overview:
    
    Authors:
    Delivery Date:
    
    File Name:  RailBot.ino
    
 *****************************************************************************
 $LOG$
 *****************************************************************************
 $NOTES$
 1) Backwards and forwards are relative to user specified intial start
 *****************************************************************************
 $REFERENCES$
 *****************************************************************************/

/*-----( ARDUINO MEGA2650 PIN CONFIGURATION )-----*/

/* LASER - UNI-T

*/


/* Radio Modules - nRF24L01
  Modules See:
  http://arduino-info.wikispaces.com/Nrf24L01-2.4GHz-HowTo
  
   1 - GND
   2 - VCC 3.3V !!! NOT 5V
   3 - CE to Arduino pin 9
   4 - CSN to Arduino pin 10
   5 - SCK to Arduino pin 13
   6 - MOSI to Arduino pin 11
   7 - MISO to Arduino pin 12
   8 - UNUSED
   
 - V1.00 11/26/13
   Based on examples at http://www.bajdi.com/
   Questions: terry@yourduino.com
*/


/*-----( Needed libraries )-----*/
#include <SPI.h>           // SPI bus Library
#include <SD.h>            // SD Library
#include <LiquidCrystal.h> // LCD Library
#include "RF24.h"          // RF Module Library
#include "printf.h"        // RF Printf Library
#include "nRF24L01.h"      // RF Module Definitions

/*-----( Pin Definitions )-----*/
// Digital Components
#define SD_CS_P                    41 // SD Chip Select (out) => pin49
#define RF_CS_P                    48 // RF Chip Select (out) => pin48
#define RF_CSN_P                   47 // RF_ (out) => pin47
#define LASER_ON_P                 43 // Laser on (out) => pin51
#define LASER_OFF_P                44 // Laser off (out) => pin50

// Analong Components
#define MOTOR_A_P                  46 // PWM A (out) => pin52
#define MOTOR_B_P                  45 // PWM B (out) => pin53
#define MOTOR_EN_P                 13 // Motor enable => pin13

// LCD Components
#define BUTTON_ADC_P           A8  // A0 is the button ADC input
//#define LCD_BACKLIGHT_P         10  // D10 controls LCD backlight
//return values for ReadButtons()
#define BUTTON_NONE                 0 // 
#define BUTTON_RIGHT                1 // 
#define BUTTON_UP                   2 // 
#define BUTTON_DOWN                 3 // 
#define BUTTON_LEFT                 4 // 
#define BUTTON_SELECT               5 // 
// ADC readings expected for the 5 buttons on the ADC input
#define RIGHT_10BIT_ADC             0 // right   
#define UP_10BIT_ADC              145 // up
#define DOWN_10BIT_ADC            329 // down
#define LEFT_10BIT_ADC            505 // left
#define SELECT_10BIT_ADC          740 // select
#define BUTTONHYSTERESIS           10 // hysteresis for valid button sensing window

// Other Deinitions
#define SHORT_PRESS               400 // Button press short (ms)
#define LONG_PRESS               1500 // Button press long (ms)
#define MAX_SPEED                 220 // Motor max PWM (count)
#define MOTOR_SPEED               220 // Normal motor speed (pwm)
#define MOTOR_ADJ_SPEED           200 // Ajusting motor speed (pwm)
#define ENCODER_DIA_INCHES (PI*1.625) // Diameter of encoder in inches (inches)


/*-----( Global Variables )-----*/
// RF Control
const uint64_t pipe = 0xF0F0F0F0D2LL; // Define the transmit pipe **LL IS LONGLONG**

// SD Control
File myFile;

// Motor Control
long count;                           // Current number of interrupts from encoder
long currentCount;                    // Position at beginning of call to move()
long countIncrement;                  // Number of counts per user interval spec
long currPosition = 1;                // Position of the robot currently (measurement number)
long totalCount;
long loopCount;                       // Number of times count increment met
int  motorDirection;                   // Direction the robot is moving
volatile boolean isMoving;            // Whether or not robot is currently moving. Declared as volatile so
                                      // it doesn't get optimized out by compiler

// LCD Control
int  currentMenu;                     // Menu to be displayed to user
long incrementFeet;                   // Feet per increment
long incrementInches;                 // & inches
long totalLengthFeet;               // Total feet of survey
long totalLengthInches;             // & inches

// Button Control
byte buttonJustPressed  = false;         //this will be true after a ReadButtons() call if triggered
byte buttonJustReleased = false;         //this will be true after a ReadButtons() call if triggered
byte buttonWas          = BUTTON_NONE;   //used by ReadButtons() for detection of button events
boolean surveyStarted = false;           // Survey Initialization

// Laser Control
boolean isMeasureing;

// Debug/Testing
int testCounter;                                      
int feet;
int inches;

/*-----( Instantiate Radio )-----*/
RF24 radio(RF_CS_P,RF_CSN_P); // Create a Radio

/*-----( Instantiate LCD )-----*/
LiquidCrystal lcd(38,39,34,35,36,37);


/*-----( ADRDUINO FUNCTIONS )-----*/
/*
  Setup: Set up all required I/O and global variables
*/
void setup()
{    
  /* Laser initialization */
  Serial.begin(115200);   // runs with 115200 baud
  laserOff();
  
  // LCD initialization
  lcd.begin(16,2);
  pinMode( BUTTON_ADC_P, INPUT );         //ensure A0 is an input
  digitalWrite( BUTTON_ADC_P, LOW );      //ensure pullup is off on A0
   
  // Setup LCD parameter
  lcd.print("READING CRANE");
  lcd.setCursor(0,1);
  lcd.print("  RAIL-BOT");
  Serial.println("Printed Reading Crane Rail-bot");
  incrementFeet   = 1;
  totalLengthFeet = 1;
  surveyStarted = false;
  currentMenu = 0;
  
  // Laser pins
  pinMode( LASER_ON_P, OUTPUT );      
  pinMode( LASER_OFF_P, OUTPUT ); 
  laserOn();
 
  // Motor control globals
  count = 0;
  loopCount = 0;
  totalCount = 0;
  motorDirection = 1;

  // Motor pin modes
  pinMode( MOTOR_A_P, OUTPUT );
  pinMode( MOTOR_B_P, OUTPUT );
  pinMode( MOTOR_EN_P, OUTPUT );

  // Motor Initialize
  digitalWrite( MOTOR_A_P, HIGH );
  digitalWrite( MOTOR_B_P, LOW );
  analogWrite( MOTOR_EN_P, 0 );

  // Rotary encoder interrupt
  attachInterrupt( 2, countInt, CHANGE );
  
  // SD card Initialization
  pinMode( SD_CS_P, OUTPUT );
  Serial.println("Initializing SD card..."); // throw error if fail ***
  if ( SD.begin(SD_CS_P) )
  {
     Serial.println("SD card initialized successfully.\n");
     myFile = SD.open("test.csv", FILE_WRITE);
      if(myFile) {
        myFile.println("N, Location (ft), Measurement (ft)");
        myFile.close();
      }
  } else {
    lcd.clear();
    lcd.print("SD ERROR");
    delay(2000);
  }

  // RF Initialization
  pinMode( RF_CS_P, OUTPUT );
  pinMode( RF_CSN_P, OUTPUT );
  printf_begin();
  radio.begin();
  Serial.println("RF Module information:");
  radio.printDetails();
  
  loopCount = 0;
  
}//--( end setup )---

/*
  loop: MAIN ENTRY FOR MEGA, RUNS CONSTANTLY
*/
void loop()
{ 
  int loopEnd = totalCount / countIncrement;
  
  if(surveyStarted)
  {
    if(loopCount < loopEnd)
    {
      Serial.println("loop - Moving Forward...");
      driveMotor();
      resetMotor();
      getDist();
      Serial.println("loop - Increment traversed!");
      loopCount++;
      
      delay(2000);
    } else {
      returnToStart();
      surveyStarted = false;
      Serial.println("SURVEY DONE!");
    }
  } else {
    displayMenu();
  }
}//--( end main loop )---


/*-----( USER FUNCTIONS )-----*/

/*
  Display Menu - 
*/
void displayMenu()
{ 
  volatile byte button = BUTTON_NONE;
  
  // Nagigate menu based on current state
  switch(currentMenu)
  {
    // Print welcome, move to runway length
    case 0: 
            delay(1500);
            lcd.clear();
            currentMenu = 1;
            break;
      
    // Display runway length while accepting input
    //  move to resolution
    case 1: lcd.clear();
      // Print the screen
      lcd.setCursor(0,0);
      lcd.print("Runway Length: ");
      lcd.setCursor(0,1);
      lcd.print(totalLengthFeet);
      lcd.print(" ft ");
      lcd.setCursor(0,1);
      setLength();
      totalCount = feet2count(totalLengthFeet);
      Serial.print("Total counts = ");
      Serial.println(totalCount);
      currentMenu = 2;
      delay(1000);
      break;
      
    // Display resolution while accepting input
    //  move to start survey
    case 2: lcd.clear();
            lcd.print("Resolution: ");
            lcd.setCursor(0,1);
            lcd.print(incrementFeet);
            lcd.print(" ft");
            setResolution();
            countIncrement = feet2count(incrementFeet);
            Serial.print("Count Increment = ");
            Serial.println(countIncrement);
            currentMenu = 3;
            delay(1000);
            break;
    // Wait for user to start survey
    case 3: lcd.clear();
            lcd.print("Start Survey");
            lcd.setCursor(0,1);
            lcd.print("Press Select");
            
            while (button != BUTTON_SELECT) {
               button = ReadButtons();
               delay(100);
            }            
            
            surveyStarted = true;
            
            lcd.clear();
            lcd.print("Survey Started...");
            
            currentMenu = 4;
            delay(2000);
            break;
    // Survey complete
    case 4: 
      lcd.setCursor(0,0);
      lcd.print("Survey Complete!");
      lcd.setCursor(0,1);
      lcd.print("Remove SD Card");
      break;
     /*for (int positionCounter = 0; positionCounter < 13; positionCounter++)
      {
        // scroll one position left:
        lcd.scrollDisplayLeft();
      // wait a bit:
      delay(300);
     
      }
      */
  }
}

void setLength()
{
  byte button = BUTTON_NONE;
  while(button != BUTTON_SELECT)
  {
    
   button = ReadButtons();
   
   switch(button)
   {
     case BUTTON_UP:
     {
       totalLengthFeet++;
       //Serial.println(totalLengthFeet);
       break;
     }
     case BUTTON_DOWN:
     {
       totalLengthFeet--; 
        //Serial.println(totalLengthFeet);
       break;
     }
     case BUTTON_SELECT:
       break;
    }
  
    // totalLengthFeet loops around limits
    if(totalLengthFeet < 1)
    {
      totalLengthFeet = 300;
    }
    else if(totalLengthFeet > 300)
    {
      totalLengthFeet = 1; 
    }
    
    // Update the display
  // if(totalLengthFeet < 10)
   // {
      lcd.setCursor(0,1);
      lcd.print("            ");
      lcd.setCursor(0,1);
      lcd.print(totalLengthFeet);
      lcd.print(" ft ");
      /*
    }
    else if (totalLengthFeet >=10 & totalLengthFeet <100)
    {
      lcd.setCursor(1,1);
      lcd.print(totalLengthFeet);
      lcd.print(" ft ");
      //lcd.print(totalLengthInches);
      //lcd.print(" inches");
      //lcd.setCursor(1,1); lcd.blink();      
    }
    else if (totalLengthFeet >= 100)
    {
      lcd.setCursor(2,1);
      lcd.print(totalLengthFeet);
      lcd.print(" ft ");
      //lcd.print(totalLengthInches);
      //lcd.print(" inches");
      //lcd.setCursor(0,2); lcd.blink();
    }
    */
    delay(100);
  }
  
  
  // continue here
}

void setResolution()
{
  byte button = BUTTON_NONE;
  while(button != BUTTON_SELECT)
  {
    
   button = ReadButtons();
   switch(button)
   {
     case BUTTON_UP:
     {
       incrementFeet++;
       //Serial.println(incrementFeet);
       break;
     }
     case BUTTON_DOWN:
     {
       incrementFeet--; 
       // Serial.println(incrementFeet);
       break;
     }
     case BUTTON_SELECT:
       break;
    }
  
    // totalLengthFeet loops around limits
    if(incrementFeet < 1)
    {
      incrementFeet = 5;
    }
    else if(incrementFeet > 5)
    {
      incrementFeet = 1; 
    }
    
    // Update the display
  // if(totalLengthFeet < 10)
   // {
      lcd.setCursor(0,1);
      lcd.print("            ");
      lcd.setCursor(0,1);
      lcd.print(incrementFeet);
      lcd.print(" ft ");
      
    delay(100);
  }
  
}

long feet2count(long feet)  //75.257 37.6286
{
   float count_estimation;
   count_estimation = (feet*12) * (16/(ENCODER_DIA_INCHES));
   //count = floor(count_estimation);
   //Serial.print("Total inches = ");
   //Serial.print(count_estimation);
   //Serial.print("Total counts = ");
   //Serial.println(countIncrement);
   //countRemainder = count_estimation - (int)count_estimation;
   return floor(count_estimation); 
}



/*--------------------------------------------------------------------------------------
  ReadButtons()
  Detect the button pressed and return the value
  Uses global values buttonWas, buttonJustPressed, buttonJustReleased.
--------------------------------------------------------------------------------------*/
byte ReadButtons()
{
   unsigned int buttonVoltage;
   byte button = BUTTON_NONE;   // return no button pressed if the below checks don't write to btn
 
   //read the button ADC pin voltage
   buttonVoltage = analogRead( BUTTON_ADC_P );
   
   //sense if the voltage falls within valid voltage windows
   if( buttonVoltage < ( RIGHT_10BIT_ADC + BUTTONHYSTERESIS ) )
   {
      button = BUTTON_RIGHT;
   }
   else if(   buttonVoltage >= ( UP_10BIT_ADC - BUTTONHYSTERESIS )
           && buttonVoltage <= ( UP_10BIT_ADC + BUTTONHYSTERESIS ) )
   {
      button = BUTTON_UP;
   }
   else if(   buttonVoltage >= ( DOWN_10BIT_ADC - BUTTONHYSTERESIS )
           && buttonVoltage <= ( DOWN_10BIT_ADC + BUTTONHYSTERESIS ) )
   {
      button = BUTTON_DOWN;
   }
   else if(   buttonVoltage >= ( LEFT_10BIT_ADC - BUTTONHYSTERESIS )
           && buttonVoltage <= ( LEFT_10BIT_ADC + BUTTONHYSTERESIS ) )
   {
      button = BUTTON_LEFT;
   }
   else if(   buttonVoltage >= ( SELECT_10BIT_ADC - BUTTONHYSTERESIS )
           && buttonVoltage <= ( SELECT_10BIT_ADC + BUTTONHYSTERESIS ) )
   {
      button = BUTTON_SELECT;
   }
   
   
   //handle button flags for just pressed and just released events
   if( ( buttonWas == BUTTON_NONE ) && ( button != BUTTON_NONE ) )
   {
      //the button was just pressed, set buttonJustPressed, this can optionally be used to trigger a once-off action for a button press event
      //it's the duty of the receiver to clear these flags if it wants to detect a new button change event
      buttonJustPressed  = true;
      buttonJustReleased = false;
   }
   if( ( buttonWas != BUTTON_NONE ) && ( button == BUTTON_NONE ) )
   {
      buttonJustPressed  = false;
      buttonJustReleased = true;
   }
 
   //save the latest button value, for change event detection next time round
   buttonWas = button;
 
   return( button );
}

/*
  Drive Motor: An algorithm to efficiently get the robot to the next position. This
    function moves the robot the distance represented by the value in countIncrement.
    This variable will be set during the initialization based on the increment distance
    configured by the user before starting the survey. The idea of this function is to
    set the speed to MOTOR_SPEED. The robot will move until the countIncrement is met.
    At this point, it is checked if the robot has made it to the correct distance. If
    There is any overshoot or undershoot, the robot will auto correct until it reaches
    the correct location.
*/
void driveMotor ()
{ 
  // Declare/Initialize adjustment parameters
  int temp_speed = MOTOR_SPEED;
  int temp_direction = motorDirection;
  
  // Save the current position
  currentCount = count;
  int targetLocation = currentCount + countIncrement;
  
  // debug
  Serial.print("driveMotor - Saving count: Current count = ");
  Serial.println(currentCount);
  
  // Drive the next location
  do
  {
    // debug
    Serial.print("driveMotor - Moving with speed: ");
    Serial.println(temp_speed);
      
    // Set normal speed
    analogWrite( MOTOR_EN_P, temp_speed );
    
    // Robot is now moving
    isMoving = true;
        
    // Begin counting traversal
    while( isMoving ); // Wait until ISR stops robot
    delay(1000);     // Slowdown time
      
    Serial.println("driveMotor - Done Moving.");
      
    // After robot has stopped, check if not at correct location
    if( count != targetLocation )
    {
      //debug
      Serial.println("driveMotor - Not at the target location");
        
      // Set adjustment speed (slower then normal speed)
      temp_speed = MOTOR_ADJ_SPEED;
        
      // Check if overshoot, then must change direction 
      if(count > targetLocation & temp_direction == motorDirection)
      {
        Serial.println("driveMotor - ...overshot it");
        changeDirection();
      }
      // Overshot while trying to adjust
      else if (count < targetLocation & temp_direction == -motorDirection)
      {
        Serial.println("driveMotor - ...overshot it");
        changeDirection();  
      }
    }
    // Found the correct location
    else
    {
     Serial.println("Found the location!!");
     // Verify direction is still forward
     if(motorDirection != temp_direction)
     {
       Serial.println("Changing direction after adjusting in opposite direction");
       changeDirection();
     }
     break; 
    }
      
  } while(true);
  
  return;
}



/*
  Change Direction: Reconfigures the motor to drive the opposite direction
*/
void changeDirection()
{
  Serial.println("changeDirection - Changing Direction...");
  analogWrite( MOTOR_EN_P, 0 );
  
  // Already moving forward, so..
  if ( motorDirection == 1 )
  {
    // Set to move backwards
    motorDirection = -1;
    digitalWrite( MOTOR_A_P, LOW );
    digitalWrite( MOTOR_B_P, HIGH );
    Serial.print("changeDirection - Motor direction switched = ");
    Serial.println(motorDirection);
  }
  // Already moving backward, so...
  else if ( motorDirection == -1 )
  {
    // Set to move forwards
    motorDirection = 1;
    digitalWrite( MOTOR_A_P, HIGH );
    digitalWrite( MOTOR_B_P, LOW );
    Serial.print("changeDirection - Motor direction switched = ");
    Serial.println(motorDirection);
  }
}

// Turns off motor and restores it to its original direction before the ISR braked it
void resetMotor() {
  
  delay(100);
  
  // Turn off motor
  analogWrite( MOTOR_EN_P, 0 );
  
  if ( motorDirection == 1 )
  {
    digitalWrite( MOTOR_A_P, HIGH );
    digitalWrite( MOTOR_B_P, LOW );
  }
  else if ( motorDirection == -1 )
  {
    digitalWrite( MOTOR_A_P, LOW );
    digitalWrite( MOTOR_B_P, HIGH );
  }
  
}

void returnToStart() {
  
  if (motorDirection == 1) {
    changeDirection();
  }
  
  currentCount = count;
  countIncrement = -(totalCount - 10);

  analogWrite( MOTOR_EN_P, MOTOR_SPEED );
  
  isMoving = true;
  
  while(isMoving);
  
  return;
  
}

//ISR for the rotary encoder
//Increments the counter variable by 1 every time it is triggered on both the rising
//and falling edge of the signal. This translates into 16 interrupts per rotation of the 
//disc. With a 1 5/8 diameter wheel, this equals 0.319068" per interrupt (1 5/8 * pi / 16)
//Brakes motor once desired distance was traveled
void countInt(){

  // Moving forward, increment counter
  if ( motorDirection == 1 )
  {
    count++;
  }
  // Moving backward, becrement counter
  else if ( motorDirection == -1 )
  {
    count--;
  }
  
  //Serial.print("CountInt - Count changed: Count = ");
  //Serial.println(count);  
  
  // Service routine to stop the robot
  if ( count == (currentCount + countIncrement) )
  { 
    Serial.println("ISR Stopping motor");
    // Toggle one motor pin to make both either 0 or 1
    digitalWrite(MOTOR_A_P, !digitalRead( MOTOR_A_P ) );
    
    // Brake motor
    analogWrite(MOTOR_EN_P, 255);
    
    // Robot is no longer moving
    isMoving = false;
   }
}


/*
  Laser On: Pulse Mega pin 7 to turn on
*/
void laserOn()
{
  // Pulse IO to tigger on
  digitalWrite(LASER_ON_P, HIGH); 
  delay(300);
  digitalWrite(LASER_ON_P, LOW);
  //Serial.println("laserOn - Laser turned on");
  delay(500);
  return;
}

/*
  Laser Off: Pulse Mega pin 8 to turn off
*/
void laserOff()
{
  // Pulse IO to trigger off
  digitalWrite(LASER_OFF_P, HIGH); 
  delay(LONG_PRESS);
  digitalWrite(LASER_OFF_P, LOW);
  //Serial.println("laserOff - Laser turned off");
  delay(500);
  return;
}

/*
  Start Measureing:
*/
void startMeasureing()
{
  // Hold laser on to begin continuous measurements
  digitalWrite(LASER_ON_P, HIGH); 
  delay(1000);
  digitalWrite(LASER_ON_P, LOW);
  
  isMeasureing = 1;
  Serial.println("startMeasureing - Taking measurement...");
  delay(500);
  return;
}

/*
  Stop Measureing: 
*/
void stopMeasureing()
{
  // Pulse laser on to stop measurments
  digitalWrite(LASER_ON_P, HIGH); 
  delay(100);
  digitalWrite(LASER_ON_P, LOW);
  
  isMeasureing = 0;
  Serial.println("stopMeasureing - Finished measurement!");
  delay(500);
  return;  
}


/*
  Get Distance: Retreives distance from Rx, stores to SD card
*/
void getDist(void){
  String dist_m;
  String dist_mm;
  String temp;
  char buf[32];
  int rc = 0;
  
  // debug
  //Serial.println("GetDist - Reading from Rx...");
  
  startMeasureing();

  delay(50);
  
  while(true)
  {
  //do
 // {
    // RC will be 21 with valid data
    rc = Serial.readBytesUntil('#', buf, sizeof(buf));
    //Serial.print("rc = ");
    //Serial.println(rc);
    //Serial.print("buf = ");
    //Serial.println(buf);
    /* // Check for Error
    if( rc == 200 )
    {
      
    }*/
    
    // Wait until vaild read
  //} while( rc != 21 );
  //stopMeasureing();
  //Serial.println("GetDist - Done reading from Rx!!");

  // Only if valid data
    if (rc == 21){
      
      // Stop taking measurements
      stopMeasureing();
          
      buf[rc] = '\0';
      temp = String(buf);
      dist_m = temp.substring(14,16);  
      dist_mm = temp.substring(16,19);
    
   
      // Print formatting
      Serial.print("Distance: ");
      Serial.print(dist_m);
      Serial.print('.');
      Serial.print(dist_mm);
      Serial.println(" m");
   
     //NEED TO ACTUALLY CONVERT METERS TO FEET FOR CSV FILE

     myFile = SD.open("test.csv", FILE_WRITE);
      if(myFile) {
        Serial.println("Writing to card");
        myFile.print(currPosition);
        myFile.print(", ");
        myFile.print(incrementFeet*currPosition);
        myFile.print(", ");
        myFile.print(dist_m);
        myFile.print('.');
        myFile.println(dist_mm);
        myFile.close();
      }
      currPosition++;
      return;
     }
   }  
  
    // debug
    //Serial.println("GetDist - Done reading, wrote to SD Card");
  return;
}

