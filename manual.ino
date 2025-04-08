#include <Servo.h>
#include <AccelStepper.h>
#include <MultiStepper.h>
//joystick definitions
#define JoystickX1 A0
#define JoystickY1 A1
//test if we need sw pins or not, no code using them other than defining joystick
#define SW10 10 
#define SW11 11
#define step_pin 8  
// Pin 3 connected to Steps pin on EasyDriver
#define dir_pin 9   
// Pin 2 connected to Direction pin
#define MS1 11      
// Pin 5 connected to MS1 pin
#define MS2 12      
#define Y_pin A2    
#define X_pin A3
#define switchL 13
#define switchR 10

Joystick joystick1(JoystickX1, JoystickY1, SW10);   //orange,green,yellow,green,blue
Joystick joystick2(X_pin, Y_pin, SW11);  //brown,red,white,gray,purple
int xpos1 = 45;
int ypos1 = 0;
int xpos2 = 69;
int x_axis1;
int y_axis1;
int x_axis2;
int servo_val0;
int servo_val1;
int servo_val2;
int direction;     // Variable to set Rotation (CW-CCW) of the motor
int steps = 1025;  // Assumes the belt clip is in the Middle
int interval = 5;
int here;
//what will be looped in manual mode
int firstloop = 0;

void mloop(int xx) {
  //gripper toggle switch for manual
  //case trial
if (firstloop == 0) {
  ServoRA2.write(45);
  ServoRA3.write(0);
  Stabilizer.write(69);
  firstloop += 1;
}

  
 Serial.print("Servo_RA2: ");
 Serial.println(ServoRA2.read());
 Serial.print("Servo_RA3: ");
 Serial.println(ServoRA3.read());
 Serial.print("Servo_Stabilizer: ");
 Serial.println(Stabilizer.read());
 Serial.print("RA1: ");
 Serial.println(stepperX.currentPosition());
  
  if (digitalRead(13) == LOW){Gripper1.write(180);}
  else { 
    if (analogRead(A5) > 1000){Gripper1.write(0);}
  else if (analogRead(A5) < 15) {Gripper1.write(90);}
  }
  x_axis1 = analogRead(A0);
  servo_val0 = map(x_axis1, 0, 1023, 0, 180);
  if (servo_val0 < 30) {
    if (xpos1 <= 0) {
    } else {
      xpos1 = xpos1 - interval;
    }
    ServoRA2.write(xpos1);
  } else if (servo_val0 > 150) {
    xpos1 = xpos1 + interval;
    ServoRA2.write(xpos1);
  }
  y_axis1 = analogRead(A1);
  servo_val1 = map(y_axis1, 0, 1023, 0, 180);
  if (servo_val1 < 30) {
    if (ypos1 <= 0) {
    } else {
      ypos1 = ypos1 - interval;
    }
    ServoRA3.write(ypos1);
  } else if (servo_val1 > 100) {
    if (ypos1 >= 180) {}
    else{
    ypos1 = ypos1 + interval;
    ServoRA3.write(ypos1);}
  }

x_axis2 = analogRead(A3);
  servo_val2 = map(x_axis2, 0, 1023, 0, 180);
  if (servo_val2 < 50) {
    if (xpos2 <= 0) {
    } else {
      xpos2 = xpos2 - interval;
    }
    Stabilizer.write(xpos2);
  } else if (servo_val2 > 100) {
    if (xpos2 >= 150) {}
    else{
    xpos2 = xpos2 + interval;
    Stabilizer.write(xpos2);}
  }  
 while (analogRead(Y_pin) >= 0 && analogRead(Y_pin) <= 100) {
    if (steps > 0) {
      digitalWrite(dir_pin, HIGH);  // (HIGH = anti-clockwise / LOW = clockwise)
      digitalWrite(step_pin, HIGH);
      delay(5);
      digitalWrite(step_pin, LOW);
      delay(5);
      steps--;
    }
  }

  while (analogRead(Y_pin) > 100 && analogRead(Y_pin) <= 400) {
    if (steps < 512) {
      digitalWrite(dir_pin, LOW);  // (HIGH = anti-clockwise / LOW = clockwise)
      digitalWrite(step_pin, HIGH);
      delay(5);
      digitalWrite(step_pin, LOW);
      delay(5);
      steps++;
    }
    if (steps > 512) {
      digitalWrite(dir_pin, HIGH);
      digitalWrite(step_pin, HIGH);
      delay(5);
      digitalWrite(step_pin, LOW);
      delay(5);
      steps--;
    }
  }


while (analogRead(Y_pin) > 601 && analogRead(Y_pin) <= 900) {
  if (steps < 1535) {
    digitalWrite(dir_pin, LOW);
    digitalWrite(step_pin, HIGH);
    delay(5);
    digitalWrite(step_pin, LOW);
    delay(5);
    steps++;
  }
  if (steps > 1535) {
    digitalWrite(dir_pin, HIGH);
    digitalWrite(step_pin, HIGH);
    delay(5);
    digitalWrite(step_pin, LOW);
    delay(5);
    steps--;
  }
}

while (analogRead(Y_pin) > 900 && analogRead(Y_pin) <= 1024) {
  if (steps < 2050) {
    digitalWrite(dir_pin, LOW);
    digitalWrite(step_pin, HIGH);
    delay(5);
    digitalWrite(step_pin, LOW);
    delay(5);
    steps++;
  }
} 
 
  delay(100);
  }
