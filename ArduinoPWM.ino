/*******************************************************************************
* PWM_Full_Hbridge.ino
*
*  Arduino sketch to utilize the SN754410 H-Bridge in order to control two
*  motors on a robotics test platform. The pin assignments should work for
*  both the Mega2560 and the Uno. Both units run at the same CPU speed (16 MHz)
*  but the Mega has 256KB of flash while the Uno has only 32KB. Also, the Uno
*  has only one hardware UART port, which might be problematic if you need to
*  receive UART motor control data from the BBB while also using the UART to
*  print DEBUG messages at run-time. Therefore it might be best to develop/test
*  the code on a Mega, and then (code size permitting) deploy to the Uno. The
*  number of PWM pins will also help make this determination as well, as the Uno
*  has only 6, while the Mega has at least twice that number.
*
*******************************************************************************/

//TODO: Need to implement the ability to support differential steering through asymmetric 
//        wheel velocities

//TODO: Look at the TimerOne library for generating PWM signals with a finer degree of control 
//        than that of the (fixed) 500Hz standard PWM capability of the Arduino Uno
//      See: http://playground.arduino.cc/Code/Timer1
//      See: Chapter 3 of Monk's "Programming Arduino Next Steps..." book

//TODO: An idea about differential turning might be allow some value of the y-axis without 
//        turning. For example if the yVal is [362,462] or [563,663], then just treat this as
//        pure motion along the X-axis. Once the y-values get outside those ranges, then maybe
//        we can start to add in differential wheel motion in the mid-ranges. This will have
//        to be studied a bit more before trying various approaches to the problem.

#define DEBUG

const int INPUT_SIZE = 20;

const int EN_LEFT = 9;     // Left side half-bridge enable
const int EN_RIGHT = 11;   // Right side half-bridge enable
const int MC1 = 3;         // Left motor control 1: Arduino pin 3 to H-bridge 1A
const int MC2 = 2;         // Left motor control 2: Arduino pin 2 to H-bridge 2A

//TODO: These two wires may need to be reversed, based upon the orientation of the motors
//       when mounted in the chassis of the robot/ROV. This will need testing. 
const int MC3 = 4;         // Right motor control 3: Arduino pin 4 to H-bridge 3A
const int MC4 = 5;         // Right motor control 4: Arduino pin 5 to H-bridge 4A

//TODO: Will need to implement differential wheel velocities
int desired_velocity = 0;  // for desired velocity

// Make sure the code starts running with the motors in the dead band
int xVal = 512;
int xValLast = 512;
int yVal = 512;
int yValLast = 512;

//TODO: These are only needed if we use the non-String-based Serial.read() methods in the 
//        main program loop (see below)
char chArray[INPUT_SIZE];
char* xToken;
char* yToken;

// Function prototypes
void left_wheel_fwd(int vel);
void left_wheel_rev(int vel);
void left_wheel_brake();
void right_wheel_fwd(int vel);
void right_wheel_rev(int vel);
void right_wheel_brake();


void setup()
{
    //TODO: May need to change this to Serial1 if we use the Arduino Mega. The Uno has only
    //       one hardware UART while the Mega has four--but the Mega's Serial port is the only
    //       one of the four that can be accessed over the USB port.
    Serial.begin(9600);
    
    // This is the solution to the Serial.read() (and related methods) blocking issue. These 
    //  methods are blocking, and have a default time-out of 1000ms. So they will block for 
    //  1 second unless the _timeout value (in the Stream.h header) is set differently, using 
    //  this public interface function. This seems to now have resolved the pause issue!
    Serial.setTimeout(100);

    // Set up both sides of the H-bridge
    pinMode(EN_LEFT, OUTPUT);
    pinMode(MC1, OUTPUT);
    pinMode(MC2, OUTPUT);
    pinMode(EN_RIGHT, OUTPUT);
    pinMode(MC3, OUTPUT);
    pinMode(MC4, OUTPUT);

    // Make sure both motors are initialized with brakes on
    left_wheel_brake();
    right_wheel_brake();
}


void loop()
{  
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
    
    // We need some logic here in case the strtok() returns NULL, which would result in some 
    //  undefined behavior from atoi(). If xToek is NULL, then it won't get changed from the 
    //  previous value.
    if (xToken != NULL) {
        xVal = atoi(xToken); 
    }
  
 //TODO:  
    // If atoi() returns a zero (0), it could mean either that the user sent a 0, or that atoi()
    //  function read an invalid value and returned a 0 to indicate this--and there's way to 
    //  tell which has occurred. Therefore we're going to have to disallow 0 as a value sent 
    //  from the CC, by limiting the range of usable values to (0,1023].
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
    
    // NOTE: Although this code is simpler than that above as it uses String objects, 
    //  some forum posters advise that the use of Strings can be problematic due to 
    //  dynamic memory management under the hood. Therefore we may need to stress-test
    //  this version, as opposed to using the char[] as noted above.
//    String xString = Serial.readStringUntil(',');
//    xVal = xString.toInt();
//    String yString = Serial.readStringUntil('\0');
//    yVal = yString.toInt();  
         
         
#ifdef DEBUG
    Serial.print("x-val: ");
    Serial.print(xVal);
    Serial.print("\ty-val: ");
    Serial.println(yVal);
#endif


// Go forward if xVal is (537,1023]
    if (xVal > 537 && xVal <= 1023) {
        desired_velocity = map(xVal, 538, 1023, 0, 255);
        left_wheel_fwd(desired_velocity);
        right_wheel_rev(desired_velocity);
    }
    // Reverse if xVal is [0, 487)
    else if (xVal >= 0 && xVal < 487) {
        desired_velocity = map(xVal, 486, 0, 0, 255);
        left_wheel_rev(desired_velocity);
        right_wheel_fwd(desired_velocity);
    }
    // Otherwise, apply the brakes
    else {
        left_wheel_brake();
        right_wheel_brake();
    }

     // Run this loop every 200ms (5Hz) for testing purposes
    //delay(50);
}

void left_wheel_fwd(int rate)
{
    // Turn off the H-bridge by grounding the enable (EN) pin. This essentially removes
    //  any voltage from the base of the transistor.
    digitalWrite(EN_LEFT, LOW);

    // Now we turn on one controller and turn the other off
    digitalWrite(MC1, HIGH);
    digitalWrite(MC2, LOW);

    // Set the PWM value for the motor control duty cycle
    analogWrite(EN_LEFT, rate);
}

void left_wheel_rev(int rate)
{
    // Turn off the H-bridge by grounding the enable (EN) pin.
    digitalWrite(EN_LEFT, LOW);

    // Now we turn on one controller and turn the other off
    digitalWrite(MC1, LOW);
    digitalWrite(MC2, HIGH);

    // Set the PWM value for the motor control duty cycle
    analogWrite(EN_LEFT, rate);
}

void right_wheel_fwd(int rate)
{
    // Turn off the H-bridge by grounding the enable (EN) pin. This essentially removes
    //  any voltage from the base of the transistor.
    digitalWrite(EN_RIGHT, LOW);

    // Now we turn on one controller and turn the other off
    digitalWrite(MC4, HIGH);
    digitalWrite(MC3, LOW);

    // Set the PWM value for the motor control duty cycle
    analogWrite(EN_RIGHT, rate);
}

void right_wheel_rev(int rate)
{
    // Turn off the H-bridge by grounding the enable (EN) pin.
    digitalWrite(EN_RIGHT, LOW);

    // Now we turn on one controller and turn the other off
    digitalWrite(MC4, LOW);
    digitalWrite(MC3, HIGH);

    // Set the PWM value for the motor control duty cycle
    analogWrite(EN_RIGHT, rate);
}

void left_wheel_brake()
{
    // Turn off H-bridge and both controllers
    digitalWrite(EN_LEFT, LOW);
    digitalWrite(MC1, LOW);
    digitalWrite(MC2, LOW);

    // Finally, saturate both transistors to apply the brakes
    digitalWrite(EN_LEFT, HIGH);
}

void right_wheel_brake()
{
    // Turn off H-bridge and both controllers
    digitalWrite(EN_RIGHT, LOW);
    digitalWrite(MC4, LOW);
    digitalWrite(MC3, LOW);

    // Saturate both transistors to apply the brakes
    digitalWrite(EN_RIGHT, HIGH);
}
