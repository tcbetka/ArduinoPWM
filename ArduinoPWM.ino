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

//TODO: Replace these when remote control is implemented
//const int POT = A0;        // Analog input for potentiometer
//long input_val = 512;

//TODO: Will need to implement differential wheel velocities
int desired_velocity = 0;  // for desired velocity
int xVal;
int yVal;

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
    char chArray[INPUT_SIZE];
    char* xToken;
    char* yToken;
    
    // Block until there is serail data available
    while (Serial.available() == 0) 
        ;
        
//TODO: This might need to be changed, as the Serial.readBytes() method seems to take a 
//        second or so to execute
    // Load the command array, get the number of bytes read and then 
    //  terminate the array
    byte size = Serial.readBytes(chArray, INPUT_SIZE);
    chArray[size] = 0;
        
    xToken = strtok(chArray, ",");
    xVal = atoi(xToken);
        
    // Use NULL for the first arg, so that we keep going in the array
    yToken = strtok(NULL, "\0");
    yVal = atoi(yToken);
    
    // Alternate method, using String class objects
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
    if (xVal > 562 && xVal <= 1023) {
        desired_velocity = map(xVal, 563, 1023, 0, 255);
        left_wheel_fwd(desired_velocity);
        right_wheel_rev(desired_velocity);
    }
    // Reverse if xVal is [0, 487)
    else if (xVal >= 0 && xVal < 462) {
        desired_velocity = map(xVal, 461, 0, 0, 255);
        left_wheel_rev(desired_velocity);
        right_wheel_fwd(desired_velocity);
    }
    // Otherwise, apply the brakes
    else {
        left_wheel_brake();
        right_wheel_brake();
    }

     // Run this loop every 200ms (5Hz) for testing purposes
    delay(200);
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
