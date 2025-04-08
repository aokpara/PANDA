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
int detection = 1;         //number of times the balls are moved to destinations
int move_finished = 1;     // Used to check if move is completed
long initial_homing = -1;  // Used to Home Stepper at startup
unsigned long SPEED = 4000;

int steps1[23] = {88, 86, 46, 12, -32, -69, -100, -73, -48, -18, 12, 39, 67, 48, 27, 4, -21, -42, -63, 323, 378, 426, 455};  //theta 1 values converted over to steps (radians-->degrees/0.45)
int theta2[23] = {45, 73, 76, 78, 77, 75, 69, 62, 67, 69, 67, 66, 59, 54, 56, 57, 56, 57, 52, 98, 87, 104, 80};                                                         //theta 2 in degrees
int theta3[23] = {0, 130, 137, 143, 137, 132, 123, 106, 119, 121, 116, 122, 98, 88, 96, 99, 97, 91, 82, 79, 66, 90, 62};                                                        //theta 3 in degrees
int theta4[23] = {69, 90, 83, 93, 88, 84, 86, 51, 64, 74, 72, 89, 37, 59, 54, 40, 60, 58, 43, 0, 13, 0, 0};                                             //theta 4 in degrees
unsigned long moveStartTime = millis(); //start moving for Servos
unsigned long moveStartTime2 = millis();
//int holepos = 0;

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
  detection = 0; 


/* Configure type of Steps on EasyDriver:
    MS1 MS2 settings
    LOW LOW = Full Step 
*/
  digitalWrite(MS1, LOW);  // Configures to Full Steps
  digitalWrite(MS2, LOW);  // Configures to Full Steps
if (analogRead(A4) > 1000)
{
//Homing setup
    pinMode(home_switch, INPUT_PULLUP);
    delay(5);  // Wait for EasyDriver wake up
 //  Set Max Speed and Acceleration of each Steppers at startup for homing
    stepperX.setMaxSpeed(100.0);      // Set Max Speed of Stepper (Slower to get better accuracy)
    stepperX.setAcceleration(100.0);  // Set Acceleration of Stepper

// Start Homing procedure of Stepper Motor at startup
  Serial.print("Stepper is Homing . . . . . . . . . . . ");
  delay(1000);
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
    stepperX.setMaxSpeed(350.0);      // Set Max Speed of Stepper (Faster for regular movements)
    stepperX.setAcceleration(100.0);  // Set Acceleration of Stepper
    stepperX.moveTo(109.00);   //109 to home RA1    
    stepperX.runToPosition();
    stepperX.setCurrentPosition(0);   // Set the current position as zero for now

} else{}
}

// moves the stepper at desired speed

void S1positions(int holepos, unsigned long moveStartTime1) {
  int RA2 = ServoRA2.read();
  int RA3 = ServoRA3.read();
  int RA4 = Stabilizer.read();
  stepperX.moveTo(steps1[holepos + 1]);
  stepperX.runToPosition();
  moveStartTime1 = millis();
  if (holepos == 0) {
    SPEED = 2000;}
  else{SPEED = 700;}
while (true) {
unsigned long progress = millis() - moveStartTime1;
  if (progress <= SPEED) {
    
    long angle1 = map(progress, 0, SPEED, RA2, theta2[holepos + 1]);
    long angle2 = map(progress, 0, SPEED, RA3, theta3[holepos + 1]);
    long angle3 = map(progress, 0, SPEED, RA4, theta4[holepos + 1]);
   ServoRA2.write(angle1);
   ServoRA3.write(angle2);
   Stabilizer.write(angle3);
  }
  else{
   ServoRA2.write(theta2[holepos + 1]);
   ServoRA3.write(theta3[holepos + 1]);
   Stabilizer.write(theta4[holepos + 1]);
    return(0);
  }


}
  
}

 void S2positions(int holepos, unsigned long moveStartTime2) {
  if (holepos == 19) {
    SPEED = 2000;
  }
  else{SPEED = 700;}
  int RA2 = ServoRA2.read();
  int RA3 = ServoRA3.read();
  int RA4 = Stabilizer.read();
while (true) {
unsigned long progress = millis() - moveStartTime2;
  if (progress <= SPEED) {
    
    long angle1 = map(progress, 0, SPEED, RA2, theta2[holepos]);
    long angle2 = map(progress, 0, SPEED, RA3, theta3[holepos]);
    long angle3 = map(progress, 0, SPEED, RA4, theta4[holepos]);
   ServoRA2.write(angle1);
   ServoRA3.write(angle2);
   Stabilizer.write(angle3);
    
      
  }
  else{
    stepperX.moveTo(steps1[holepos]);
  stepperX.runToPosition();
    return(0);
  }


}
  
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

void loop() {
  unsigned long process = millis();
  int settings = analogRead(A4);
  if (settings > 1000) { 
    while(detection < 4) {
      for (int i = 0; i < 18; ++i)  //for every hole on S1, sensor will read
      {
        moveStartTime = millis();
        S1positions(i, moveStartTime);
        delay(1000);
        if (digitalRead(2) == 0)  //if ball is there
        {  
          detection += 1;
          
          Gripper('p');
          delay(1000);
          Gripper('c');
          moveStartTime2 = millis();
           if (detection == 1) {

            S2positions(19, moveStartTime2);
            Gripper('d');
            delay(500);
            Gripper('c');
            moveStartTime = millis();
            S1positions(i, moveStartTime);
          }
          else if (detection == 2) {
            S2positions(20, moveStartTime2);
            Gripper('d');
            delay(500);
            Gripper('c');
            moveStartTime = millis();
            S1positions(i, moveStartTime);
          } 
          else if (detection == 3) {
            S2positions(21, moveStartTime2);
            Gripper('d');
            delay(500);
            Gripper('c');
            moveStartTime = millis();
            S1positions(i, moveStartTime);
          } 
          else if (detection == 4) {
            S2positions(22, moveStartTime2);
            Gripper('d');
            delay(500);
            Gripper('c');
            moveStartTime = millis();
            S1positions(i,moveStartTime);
          }
          else {
            S2positions(random(19,22), moveStartTime2);
            Gripper('d');
            delay(500);
            Gripper('c');
            moveStartTime = millis();
            S1positions(i, moveStartTime);
          }
        }
     else {}
        }
    }
}
  
  else if (settings < 20) {
    int positioning = analogRead(A5);
    mloop(positioning);    
  }

}
