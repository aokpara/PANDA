//need the following libraries
#include <Servo.h>
#include <Button.h>
#include <Joystick.h>
#include <AccelStepper.h>
#include <MultiStepper.h>
/*code for auto mode vrs 2 */

//Manual Controls
#define switchA4 A4  //AUTO-ON/OFF/MANUAL-ON
#define switchA5 A5  //Gripper controls for manual mode

//object initialization
Servo Gripper1;    //gripper for magnet
Servo Stabilizer;  //stabilizer
Servo ServoRA2;
Servo ServoRA3;

//Driver for RA1 Stepper Motor
#define step_pin 8  // Pin 3 connected to Steps pin on EasyDriver
#define dir_pin 9   // Pin 2 connected to Direction pin
#define MS1 11      // Pin 5 connected to MS1 pin
#define MS2 12      // Pin 4 connected to MS2 pin
#define SLEEP 10    // Pin 7 connected to SLEEP pin
#define X_pin A2    // Pin A0 connected to joystick x axis
#define home_switch 7
AccelStepper stepperX(1, step_pin, dir_pin);

int balldistance;          //digital Read of IR sensor
int detection = 0;         //number of times the balls are moved to destinations
int move_finished = 1;     // Used to check if move is completed
long initial_homing = -1;  // Used to Home Stepper at startup

//positions of targets for auto mode
int steps1[22] = { 85, 57, 8, -35, -72, -100, -79, -46, -16, 16, 42, 66, 50, 32, 12, -11, -35, -56, 331, 402, 433, 485 };  //theta 1 values converted over to steps (radians-->degrees/0.45)
int theta2[22] = {70, 70, 72, 74, 75, 68, 61, 61, 65, 65, 65, 60, 49, 53, 54, 54, 51, 50, 89, 76, 95, 69};                 //theta 2 in degrees
int theta3[22] = { 141, 148, 145, 157, 147, 136, 119, 129, 128, 128, 140, 119, 98, 114, 110, 110, 99, 105, 89, 81, 93, 8}; //theta 3 in degrees
int theta4[22] = {69, 66, 82, 82, 83, 70, 61, 54, 62, 61, 63, 61, 46, 51, 51, 51, 47, 49, 12, 9, 2, 0};                    //theta 4 in degrees

void setup() {
  ServoRA2.attach(5);
  ServoRA3.attach(6);
  Gripper1.attach(3);
  Stabilizer.attach(4);
  Serial.begin(9600);
  pinMode(MS1, OUTPUT);
  pinMode(MS2, OUTPUT);
  pinMode(dir_pin, OUTPUT);
  pinMode(step_pin, OUTPUT);
  ServoRA2.write(45);
  ServoRA3.write(0);
  Stabilizer.write(69);

//hello world
/* Configure type of Steps on EasyDriver:
    MS1 MS2 settings
    LOW LOW = Full Step 
*/
  digitalWrite(MS1, LOW);  // Configures to Full Steps
  digitalWrite(MS2, LOW);  // Configures to Full Steps

//Homing setup
    pinMode(home_switch, INPUT_PULLUP);
    delay(5);  // Wait for EasyDriver wake up
 //  Set Max Speed and Acceleration of each Steppers at startup for homing
    stepperX.setMaxSpeed(100.0);      // Set Max Speed of Stepper (Slower to get better accuracy)
    stepperX.setAcceleration(100.0);  // Set Acceleration of Stepper

// Start Homing procedure of Stepper Motor at startup
  Serial.print("Stepper is Homing . . . . . . . . . . . ");
  while (digitalRead(home_switch)) {  // Make the Stepper move CCW until the switch is   activated
      stepperX.moveTo(initial_homing);  // Set the position to move to
      initial_homing--;                 // Decrease by 1 for next move if needed
      stepperX.run();                   // Start moving the stepper
      delay(5);
    }
    stepperX.setCurrentPosition(0);   // Set the current position as zero for now
    stepperX.setMaxSpeed(100.0);      // Set Max Speed of Stepper (Slower to get better accuracy)
    stepperX.setAcceleration(100.0);  // Set Acceleration of Stepper
    initial_homing = 1;
  
    while (!digitalRead(home_switch)) {  // Make the Stepper move CW until the switch is deactivated
      stepperX.moveTo(initial_homing);
      stepperX.run();
      initial_homing++;
      delay(5);
    }
  
    stepperX.setCurrentPosition(0);
    Serial.println("Homing Completed");
    Serial.println("");
    stepperX.setMaxSpeed(400.0);      // Set Max Speed of Stepper (Faster for regular movements)
    stepperX.setAcceleration(400.0);  // Set Acceleration of Stepper
    stepperX.moveTo(109.00);   //109 to home RA1    
    stepperX.runToPosition();
    stepperX.setCurrentPosition(0);   // Set the current position as zero for now

}

void S1positions(int holepos) {

  ServoRA2.write(theta2[holepos]);
  ServoRA3.write(theta3[holepos]);
  Stabilizer.write(theta4[holepos]);
  delay(500);
  stepperX.moveTo(steps1[holepos]);
  stepperX.runToPosition();
  
}

void Gripper(char mech) {
  if (mech == 'p') {
    Gripper1.write(0);
  }
  if (mech == 'c') {
    Gripper1.write(90);
  }
  if (mech == 'd') {
    Gripper1.write(180);
  }
}
void destination(int there)  //function to choose a random hole for 5th hole, and all destinations for 4 holes
{
  if (there == 19)  //destination hole 19
  {ServoRA2.write(theta2[18]);  //theta 2
    ServoRA3.write(theta3[18]);  //theta 3
    Stabilizer.write(theta4[18]);
    delay(1000);
    stepperX.moveTo(steps1[18]);
    stepperX.runToPosition();
    delay(1000);
    
  }
  if (there == 20)  //destination hole 20
  {ServoRA2.write(theta2[19]);  //theta 2
    ServoRA3.write(theta3[19]);  //theta 3
    Stabilizer.write(theta4[19]);
    delay(1000);
    stepperX.moveTo(steps1[19]);
      stepperX.runToPosition();
        delay(1000);
    
  }
  if (there == 21)  //destination hole 21
  {ServoRA2.write(theta2[20]);  //theta 2
    ServoRA3.write(theta3[20]);  //theta 3
    Stabilizer.write(theta4[20]);
    delay(100);
    stepperX.moveTo(steps1[20]);
    stepperX.runToPosition();
        delay(1000);
    
  }
  if (there == 22)  //destination hole 22
  {ServoRA2.write(theta2[21]);  //theta 2
    ServoRA3.write(theta3[21]);  //theta 3
    Stabilizer.write(theta4[21]);
    delay(1000);
    stepperX.moveTo(steps1[21]);
    stepperX.runToPosition();
        delay(1000);
  }
  if (there == 23)  // random destination hole
  {    int rand = random(19, 22);
ServoRA2.write(theta2[rand]);  //theta 2
    ServoRA3.write(theta3[rand]);  //theta 3
    Stabilizer.write(theta4[rand]);
    delay(1000);
    stepperX.moveTo(steps1[rand]);
    stepperX.runToPosition();
        delay(1000);
  }
}

void loop() {
  int settings = analogRead(A4);
  if (settings > 1000) {

      for (int i = 0; i < 1; ++i)  //for every hole on S1, sensor will read
      {
        S1positions(0);
      }
 
}
  else if (settings < 20) {
    int positioning = analogRead(A5);
    mloop(positioning);
  }
}
