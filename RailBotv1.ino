/*****************************************************************************

    File Overview:
    
    Authors:
    Delivery Date:
    
    File Name:  RailBot.ino
    
 *****************************************************************************
 $LOG$
 *****************************************************************************
 $NOTES$
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
#define MOTOR_A_P 25      // PWM A (out) => pin25
#define MOTOR_B_P 26      // PWM B (out) => pin26
#define MOTOR_EN_P 13     // Motor enable => pin13
#define MAX_SPEED 240     // Motor max PWM (count)
#define MOTOR_SPEED 220   // Normal motor speed

/*-----( Global Variables )-----*/
const uint64_t pipe = 0xF0F0F0F0D2LL; // Define the transmit pipe **LL IS LONGLONG**
long count;                           // Current number of interrupts from encoder
long countIncrement;                  // Number of counts per user interval spec
long loopCount;                       // Number of times count increment met
int motorDirection;                   // Direction the robot is moving
long currentCount;                    // Position at beginning of call to move()
volatile boolean moving;              // Whether or not robot is currently moving. Declared as volatile so
                                      // it doesn't get optimized out by compiler

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
  countIncrement = 50;
  motorDirection = 1;
  
  // Set up rotary encoder interrupt
  attachInterrupt(2, countInt, CHANGE);
  
  // SD card Initialization
  Serial.println("Initializing SD card..."); // throw error if fail ***
  if(SD.begin(SD_CS_P))
  {
     Serial.println("SD card initialized successfully.\n");
  }
  
  // Set up motor pin modes
  pinMode(MOTOR_A_P, OUTPUT);
  pinMode(MOTOR_B_P, OUTPUT);
  pinMode(MOTOR_EN_P, OUTPUT);

  // Motor initialization
  digitalWrite(MOTOR_A_P, HIGH);
  digitalWrite(MOTOR_B_P, LOW);
  analogWrite(MOTOR_EN_P, 0);
  
  // RF Initialization
  printf_begin();
  radio.begin();
  Serial.println("RF Module information:");
  radio.printDetails();
  
  
}//--( end setup )---

/*
  loop: MAIN ENTRY FOR MEGA, RUNS CONSTANTLY
*/
void loop()
{ 
  move();
  resetMotor();
  delay(2000);
  changeDirection();
  
  move();
  resetMotor();
  delay(2000);
  changeDirection();
  
}//--( end main loop )---


/*-----( USER FUNCTIONS )-----*/

//This function moves the robot the distance represented by the value in
//countIncrement. This variable will be set during the initialization
//based on the increment distance configured by the user before starting the survey.
//The idea of this function is to speed up to MAX_SPEED in a controlled fashion
//instead of instantly going to MAX_SPEED. The motor speed will increase up to a 
//max of MAX_SPEED until it has traversed half the required distance, and then will
//begin to slow back down to a stop for the second half.

//NOTE: play with delay values and while loop conditions once motor is functional
void move ()
{
  //save the current position
  currentCount = count;
//  int motorSpeed = 0;
  
  // Set PWM with respect to direction
  /*if (motorDirection == 1)  // Move forward
  {
    digitalWrite(MOTOR_A_P, HIGH);
    digitalWrite(MOTOR_B_P, LOW);
  }
  else if (motorDirection == -1)// Move backward
  {
    digitalWrite(MOTOR_A_P, LOW);
    digitalWrite(MOTOR_B_P, HIGH);
  }*/
  
  // Set Motor Speed
//  while (count < (currentCount + countIncrement - 20))
//  {
//speed up until half of the distance is traversed

    // Set normal speed
    analogWrite(MOTOR_EN_P, MOTOR_SPEED);
    
    // Robot is now moving
    moving = true;
    
    // Wait until ISR stops robot
    while(moving);
     
//    if(motorSpeed + 30 > MAX_SPEED)
//    {
//      motorSpeed = MAX_SPEED;
//    }
//    else
//    {
//      motorSpeed+=30;
//    }
     
  //  delay(300);
  //}
  //Serial.println("Slowing speed.");
  //motorSpeed = 200;
  //analogWrite(MOTOR_EN_P, motorSpeed);
   
  //while ((count < currentCount + countIncrement - 5)){
  //  delay(1);
  //}
  return;
}

/*
  Change Direction:
*/
void changeDirection()
{
  Serial.print("Direction = ");
  Serial.println(motorDirection);
  analogWrite(MOTOR_EN_P, 0);
  delay(50);
  // Already moving forward, so..
  if(motorDirection == 1)
  {
    // Set to move backwards
    motorDirection = -1;
    Serial.print("motor direction switched = ");
    Serial.println(motorDirection);
    digitalWrite(MOTOR_A_P, LOW);
    digitalWrite(MOTOR_B_P, HIGH);
  }
  // Already moving backward, so...
  else if(motorDirection == -1)
  {
        // Set to move backwards
    motorDirection = 1;
    digitalWrite(MOTOR_A_P, HIGH);
    digitalWrite(MOTOR_B_P, LOW);
  }
}


// Turns off motor and restores it to its original direction before the ISR braked it
void resetMotor() {
  
  delay(100);
  
  // Turn off motor
  analogWrite(MOTOR_EN_P, 0);
  
  // Restore direction before ISR braked
  digitalWrite(MOTOR_A_P, !(digitalRead(MOTOR_A_P)));
  
}

//ISR for the rotary encoder
//Increments the counter variable by 1 every time it is triggered on both the rising
//and falling edge of the signal. This translates into 16 interrupts per rotation of the 
//disc. With a 1 5/8 diameter wheel, this equals 0.38" per interrupt
//Brakes motor once desired distance was traveled
void countInt(){
  
  count++;
  
  Serial.print("Count = ");
  Serial.println(count);  
  
  // Service routine to stop the robot
  if(count == currentCount + countIncrement)
  {
    Serial.println("Braking");
    
    // Toggle one motor pin to make both either 0 or 1
    digitalWrite(MOTOR_A_P, !digitalRead(MOTOR_A_P));
    
    // Brake motor
    analogWrite(MOTOR_EN_P, 255);
    
    // Robot is no longer moving
    moving = false;
    
   }
}


