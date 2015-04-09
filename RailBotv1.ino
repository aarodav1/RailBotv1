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
#define SD_CS_P 24         // SD Chip Select (out) => pin24
#define RF_CS_P 22         // RF Chip Select (out) => pin22
#define RF_CSN_P 23        // RF_ (out) => pin23
#define LASER_ON_P = 7;    // Laser on (out) pin+7
#define LASER_OFF_P = 8;   // Laser off (out) => pin8

// Analong Components
#define MOTOR_A_P 26         // PWM A (out) => pin25
#define MOTOR_B_P 25         // PWM B (out) => pin26
#define MOTOR_EN_P 13        // Motor enable => pin13
#define MAX_SPEED 255        // Motor max PWM (count)
#define MOTOR_SPEED 255      // Normal motor speed
#define MOTOR_ADJ_SPEED 200  // Ajusting motor speed 

/*-----( Global Variables )-----*/
const uint64_t pipe = 0xF0F0F0F0D2LL; // Define the transmit pipe **LL IS LONGLONG**
long count;                           // Current number of interrupts from encoder
long currentCount;                    // Position at beginning of call to move()
long countIncrement;                  // Number of counts per user interval spec
long loopCount;                       // Number of times count increment met
int motorDirection;                   // Direction the robot is moving
volatile boolean moving;              // Whether or not robot is currently moving. Declared as volatile so
                                      // it doesn't get optimized out by compiler
                                      
int testCounter;                                      


/*-----( Instantiate Radio )-----*/
RF24 radio(RF_CS_P,RF_CSN_P); // Create a Radio


/*-----( ADRDUINO FUNCTIONS )-----*/
/*
  Setup: Set up all required I/O and global variables
*/
void setup()
{    
  // Laser initialization
  Serial.begin(115200);   // runs with 115200 baud
  
  // Assign global variables
  count = 0;
  loopCount = 0;
  countIncrement = 25;
  motorDirection = 1;
  
  // Set up rotary encoder interrupt
  attachInterrupt( 2, countInt, CHANGE );
  
  // SD card Initialization
  Serial.println("Initializing SD card..."); // throw error if fail ***
  if ( SD.begin(SD_CS_P) )
  {
     Serial.println("SD card initialized successfully.\n");
  }
  
  // Set up motor pin modes
  pinMode( MOTOR_A_P, OUTPUT );
  pinMode( MOTOR_B_P, OUTPUT );
  pinMode( MOTOR_EN_P, OUTPUT );

  // Motor initialization
  digitalWrite( MOTOR_A_P, HIGH );
  digitalWrite( MOTOR_B_P, LOW );
  analogWrite( MOTOR_EN_P, 0 );
  
  // RF Initialization
  printf_begin();
  radio.begin();
  Serial.println("RF Module information:");
  radio.printDetails();

  Serial.println("Setup - Setup compelte!");  
  
}//--( end setup )---

/*
  loop: MAIN ENTRY FOR MEGA, RUNS CONSTANTLY
*/
void loop()
{ 
  
if(testCounter <3){
  Serial.println("loop - Moving Forward...");
  driveMotor();
  //resetMotor();
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
    moving = true;
        
    // Begin counting traversal
    while( moving ); // Wait until ISR stops robot
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
  
  // Restore direction before ISR braked
  digitalWrite( MOTOR_A_P, !(digitalRead(MOTOR_A_P)) );
  
}

//ISR for the rotary encoder
//Increments the counter variable by 1 every time it is triggered on both the rising
//and falling edge of the signal. This translates into 16 interrupts per rotation of the 
//disc. With a 1 5/8 diameter wheel, this equals 0.38" per interrupt
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
  
  Serial.print("CountInt - Count increased: Count = ");
  Serial.println(count);  
  
  // Service routine to stop the robot
  if ( count == (currentCount + countIncrement) )
  { 
    // Toggle one motor pin to make both either 0 or 1
    //digitalWrite( MOTOR_A_P, !digitalRead( MOTOR_A_P ) );
    
    // Brake motor
    analogWrite( MOTOR_EN_P, 0);
    
    // Robot is no longer moving
    moving = false;
   }
}


