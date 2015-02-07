/*******************************************************************************
* Code to experiment with the Adafruit Motorshield controller, based on the
*  sample code provided by Adafruit. This device uses the i2c bus to control
*  4 DC motors or 2 servos, however these boards can be stacked to add DC
*  motors as need. This code turns out to be much simpler than that previously
*  developed for the SN754410 H-Bridge motor controller.
*******************************************************************************/
//
//TODO List:
//
//  1) Need to implement the ability to support differential steering through asymmetric 
//      wheel velocities. An idea about differential turning might be allow some value 
//      of the y-axis without turning. For example if the yVal is [362,462] or [563,663], 
//      then just treat this as pure motion along the X-axis. Once the y-values get 
//      outside those ranges, then maybe we can start to add in differential wheel motion 
//      in the mid-ranges. This will have to be studied a bit more before trying various 
//      approaches to the problem.
//
//  2) Implement the ability to sense off-course motion, based on sensor information from 
//      the wheel encoders, and then some sort of feedback based upon that sensor data.
//
//  3) Look at the TimerOne library for generating PWM signals with a finer degree of control 
//      than that of the (fixed) 500Hz standard PWM capability of the Arduino Uno.
//        See: http://playground.arduino.cc/Code/Timer1
//        See: Chapter 3 of Monk's "Programming Arduino Next Steps..." book


// Includes needed for Adafruit's motorshield
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"

#define DEBUG
//#define JOYSTICK
#define SERIAL

// Function prototypes
void lWheelStop(void);
void rWheelStop(void);

const int INPUT_SIZE = 20;

// For testing the joystick controller while attached directly to the Arduino
#ifdef JOYSTICK
    const int yRangeIn = A0;
    const int xRangeIn = A1;
#endif

// For reducing the sensitivity during differential steering
const float SCALE_FACTOR = 0.7f;

// We need a motorshield object (default address 0x60) and a couple of motor pointers. We will  
//  use channels 1 and 2 on the motor shield, and leave channels 3 and 4 open for future feature 
//  addition
Adafruit_MotorShield myShield = Adafruit_MotorShield();
Adafruit_DCMotor* lMotor = myShield.getMotor(1);
Adafruit_DCMotor* rMotor = myShield.getMotor(2);

// Make sure the code starts running with the motors in the dead band
int xVal = 512;
int xValLast = 512;
int yVal = 512;
int yValLast = 512;
int desired_velocity = 0;
int desired_turn_rate = 0;


//TODO: These are only needed if we use the non-String-based Serial.read() methods in the 
//        main program loop (see below)
#ifdef SERIAL
    char chArray[INPUT_SIZE];
    char* xToken;
    char* yToken;
#endif


void setup()
{
    //TODO: May need to change this to Serial1 if we use the Arduino Mega. The Uno has only
    //       one hardware UART while the Mega has four--but the Mega's Serial port is the only
    //       one of the four that can be accessed over the USB port.
    Serial.begin(9600);
    
    // Avoid prolonged blocking from Serial.read() and related methods that block for one second by default
    Serial.setTimeout(100);
    
#ifdef JOYSTICK
    // Set-up the analog input pins
    pinMode(xRangeIn, INPUT);
    pinMode(yRangeIn, INPUT);
#endif

    // Create the MotorShield with a default 1.6KHz frequency
    myShield.begin();
    
    // Stop both wheels
    lWheelStop();
    rWheelStop();
}


void loop()
{  
    
#if defined(JOYSTICK)
    //TODO: Remove when BBB/Arduino connection is implemented
    xVal = analogRead(xRangeIn);
    yVal = analogRead(yRangeIn);

#elif defined(SERIAL)
    //TODO: Reimplement and test this logic once the BBB/Arduino connection is in place

    // Block until there is serial data available
    while (Serial.available() == 0) 
        ;

    // Load the command array up to the first NULL and get the number of bytes read. Then 
    //  we terminate the array
    byte size = Serial.readBytesUntil('\0', chArray, INPUT_SIZE);
    chArray[size] = '\0';
    
    // The returned string should be something like 300,400
    xToken = strtok(chArray, ",");
    
    #ifdef DEBUG
        Serial.print("xToken: ");
        Serial.println(xToken);
    #endif
    
    // Prevent undefined behavior from atoi() if strtok() returns NULL 
    if (xToken != NULL) {
        xVal = atoi(xToken); 
    }
 
    //TODO: Test this logic
    // If atoi() returns a zero (0), it could mean either that the user sent a 0, or that atoi()
    //  function read an invalid value and returned a 0 to indicate this. Since there's no way 
    //  to tell which has occurred, we're going to have to disallow 0 as a value sent 
    //  from the CC by limiting the range of usable values to (0,1023].
    if (xVal == 0) { 
        xVal = xValLast;
    } else {
        xValLast = xVal;
    }
    
    // Use NULL for the first arg, so that we keep going in the array instead of starting anew
    yToken = strtok(NULL, "\0");
    
    #ifdef DEBUG
        Serial.print("yToken: ");
        Serial.println(yToken);
    #endif
 
    if (yToken != NULL) {
        yVal = atoi(yToken);
    }
    
    if (yVal == 0) {
        yVal = yValLast;
    } else {
        yValLast = yVal;
    }
#endif         
         
#ifdef DEBUG
    Serial.print("x-val: ");
    Serial.print(xVal);	
    Serial.print("\ty-val: ");
    Serial.println(yVal);
#endif

    // If the y-value is in the deadband, only go forward or backwards
    if (yVal > 487 && yVal < 538)
    {
        // FWD if xVal is (537,1023]
        if (xVal > 537 && xVal <= 1023) {         
            desired_velocity = map(xVal, 538, 1023, 0, 255);
            lMotor->setSpeed(desired_velocity);
            rMotor->setSpeed(desired_velocity);
            lMotor->run(FORWARD);
            rMotor->run(FORWARD);
            
            #ifdef DEBUG
                Serial.println("Direction: FWD");
            #endif
            
            // TODO: Give the motor a bit of time to respond to command before allowing changes. Need 
            //        to experiment with this a bit, in terms of proper values and if it's even needed.
            delay(10);
        } 
        // REV if xVal is (0, 487)
        else if (xVal >= 0 && xVal < 487) {     
            desired_velocity = map(xVal, 486, 0, 0, 255);
            lMotor->setSpeed(desired_velocity);
            rMotor->setSpeed(desired_velocity);
            lMotor->run(BACKWARD);
            rMotor->run(BACKWARD);
            
            #ifdef DEBUG
                Serial.println("Direction: REV");
            #endif
            
            delay(10);
        } 
        // Otherwise, apply the brakes
        else {   
            lWheelStop();
            rWheelStop();
            delay(10);      
        }
    }
    // Otherwise we want to turn left or right, so this takes precedence over forward/reverse
    else
    {      
        // RIGHT (Note: reduce the desired_turn_rate by a factor of a multiplier.)
        if (yVal > 537 && yVal <= 1023) {         
            desired_turn_rate = map(yVal, 538, 1023, 0, 255);
            lMotor->setSpeed((int)(SCALE_FACTOR * desired_turn_rate));          
            lMotor->run(FORWARD);
            rWheelStop(); 
            //rMotor->setSpeed(desired_turn_rate);
            //rMotor->run(BACKWARD);
            
            #ifdef DEBUG
                Serial.println("Direction: RGT");
            #endif
            
            // TODO: Give the motor a bit of time to respond to command before allowing changes. Need 
            //        to experiment with this a bit, in terms of proper values and if it's even needed.
            delay(10);
        } 
        // LEFT (Note: reduce the desired_turn_rate by a factor of a multiplier.)
        else if (yVal >= 0 && yVal < 487) {     
            desired_turn_rate = map(yVal, 486, 0, 0, 255);
            rMotor->setSpeed((int)(SCALE_FACTOR * desired_turn_rate));            
            rMotor->run(FORWARD);
            lWheelStop();
            //lMotor->setSpeed(desired_turn_rate);
            //lMotor->run(BACKWARD);
            
            #ifdef DEBUG
                Serial.println("Direction: LFT");
            #endif
            
            delay(10);
        } 
        // Otherwise, apply the brakes
        else {   
            lWheelStop();
            rWheelStop();
            delay(10);      
        }  
    }
    
    // Delay here for when using the joystick
    delay(250);
}

//TODO: Need to investigate whether or not the BRAKE needs to be applied, or if the setSpeed(0) call 
//        is adequate by itself? Also, should we have a lWheelStop() and a lWheelSlow() function, 
//        maybe to support a more gradual turn instead of stop-turn-start type of behavior?
void lWheelStop(void)
{
    lMotor->setSpeed(0);
    //lMotor->run(BRAKE);
}

void rWheelStop(void)
{
    rMotor->setSpeed(0);
    //,rMotor->run(BRAKE);  
}


