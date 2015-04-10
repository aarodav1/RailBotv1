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
#include <SPI.h>       // SPI bus Library
#include <SD.h>        // SD Library
#include "RF24.h"      // RF Module Library
#include "printf.h"    // RF Printf Library
#include "nRF24L01.h"  // RF Module Definitions

/*-----( Pin Definitions )-----*/
// Digital Components
#define SD_CS_P 24           // SD Chip Select (out) => pin24
#define RF_CS_P 22           // RF Chip Select (out) => pin22
#define RF_CSN_P 23          // RF_ (out) => pin23
#define LASER_ON_P 27        // Laser on (out) => pin25
#define LASER_OFF_P 28       // Laser off (out) => pin27

// Analong Components
#define MOTOR_A_P 26         // PWM A (out) => pin25
#define MOTOR_B_P 25         // PWM B (out) => pin26
#define MOTOR_EN_P 13        // Motor enable => pin13

// Other Deinitions
#define SHORT_PRESS 400                  // Button press short
#define LONG_PRESS 1500                  // Button press long
#define MAX_SPEED 255                    // Motor max PWM (count)
#define MOTOR_SPEED 255                  // Normal motor speed
#define MOTOR_ADJ_SPEED 200              // Ajusting motor speed 
#define ENCODER_DIA_INCHES (PI*13/8)  // Diameter of encoder in inches

/*-----( Global Variables )-----*/
// RF Control
const uint64_t pipe = 0xF0F0F0F0D2LL; // Define the transmit pipe **LL IS LONGLONG**

// SD Control
File myFile;

// Motor Control
long count;                           // Current number of interrupts from encoder
long currentCount;                    // Position at beginning of call to move()
long countIncrement;                  // Number of counts per user interval spec
long countRemainder;
long loopCount;                       // Number of times count increment met
int motorDirection;                   // Direction the robot is moving
volatile boolean isMoving;            // Whether or not robot is currently moving. Declared as volatile so
                                      // it doesn't get optimized out by compiler
// Laser Control
boolean isMeasureing;

// Debug/Testing
int testCounter;                                      
int feet;
int inches;
float percision;

/*-----( Instantiate Radio )-----*/
RF24 radio(RF_CS_P,RF_CSN_P); // Create a Radio


/*-----( ADRDUINO FUNCTIONS )-----*/
/*
  Setup: Set up all required I/O and global variables
*/
void setup()
{    
  /* Laser initialization */
  Serial.begin(115200);   // runs with 115200 baud
  
  // Laser pins
  pinMode( LASER_ON_P, OUTPUT );      
  pinMode( LASER_OFF_P, OUTPUT ); 
 
  // Motor control globals
  count = 0;
  loopCount = 0;
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
  }

  // RF Initialization
  pinMode( RF_CS_P, OUTPUT );
  pinMode( RF_CSN_P, OUTPUT );
  printf_begin();
  radio.begin();
  Serial.println("RF Module information:");
  radio.printDetails();



  // Test info
  testCounter = 0;
  feet = 2;
  inches = 6;
  percision = 0;
  
  increment2count(feet, inches, percision);
  
}//--( end setup )---

/*
  loop: MAIN ENTRY FOR MEGA, RUNS CONSTANTLY
*/
void loop()
{ 
  
if(testCounter < 2){
  Serial.println("loop - Moving Forward...");
  laserOn();
  driveMotor();
  resetMotor();
  takeMeasurement();
  laserOff();
  Serial.println("loop - Increment traversed!");
  delay(2000);
}
testCounter++;
Serial.print("Test counter: ");
Serial.println(testCounter);
  //changeDirection();
  //driveMotor();
  //resetMotor();
  //delay(2000);
  //changeDirection();
  
}//--( end main loop )---


/*-----( USER FUNCTIONS )-----*/


void increment2count(int feet, int inches, float percision)
{
   float count_estimation;
   count_estimation = (feet*12 + inches + percision) * (ENCODER_DIA_INCHES)/16;
   countIncrement = floor(count_estimation);
   Serial.print("Total inches = ");
   Serial.print(count_estimation);
   Serial.print("Total counts = ");
   Serial.println(countIncrement);
   countRemainder = count_estimation - (int)count_estimation;
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
  
  Serial.print("CountInt - Count changed: Count = ");
  Serial.println(count);  
  
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
  // Pulse IO to tigger off
  digitalWrite(LASER_ON_P, HIGH); 
  delay(SHORT_PRESS);
  digitalWrite(LASER_ON_P, LOW);
  delay(500);
  return;
}

/*
  Laser Off: Pulse Mega pin 8 to turn off
*/
void laserOff()
{
  // Pulse IO to trigger on 
  digitalWrite(LASER_OFF_P, HIGH); 
  delay(LONG_PRESS);
  digitalWrite(LASER_OFF_P, LOW);
  delay(500);
  return;
}

/*
  Stop Continuous: 
*/
void stopMeasureing()
{
  // Pulse laser on to stop measurments
  digitalWrite(LASER_ON_P, HIGH); 
  delay(SHORT_PRESS);
  digitalWrite(LASER_ON_P, LOW);
  
  isMeasureing = 0;
  delay(500);
  return;  
}

/*
  Inititiate Continuous:
*/
void startMeasureing()
{
  // Hold laser on to begin continuous measurements
  digitalWrite(LASER_ON_P, HIGH); 
  delay(1000);
  digitalWrite(LASER_ON_P, LOW);
  
  isMeasureing = 1;
  delay(500);
  return;
}


/*
  Get Distance: Retreives distance from Rx, stores to SD card
*/
void GetDist(void){
  String dist_m;
  String dist_mm;
  String temp;
  char buf[32];
  int rc = 0;
  
  // RC will be 21 with valid data
  rc = Serial.readBytesUntil('#', buf, sizeof(buf));
    
  // Only if valid data
  if (rc == 21){
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
   
    if(myFile) {
      Serial.println("Writing to card");
      myFile.print("Distance: ");
      myFile.print(dist_m);
      myFile.print('.');
      myFile.print(dist_mm);
      myFile.println(" m. Position: ");
    }
  }  
  return;
}

/*
  Test Measurement: Continuous measurement
*/
void takeMeasurement()
{
    // Start continuous measurements  
    startMeasureing();
    
    // Wait time?
    delay(25);

    // Sample the distance a few times, play with i and delays
    GetDist();
    
    // Stop taking measurements
    stopMeasureing();
    
 return;
}
