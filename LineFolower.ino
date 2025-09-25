#include "TRSensors.h"
 
#define NUM_SENSORS 5
#define MAX_SPEED 150 // Maxspeed
#define DEFAULT_SPEED 70 // Base speed
 
#define MOTOR1_PIN_DIR1 14
#define MOTOR1_PIN_DIR2 15
#define MOTOR1_PIN_PWM 5
 
#define MOTOR2_PIN_DIR1 16
#define MOTOR2_PIN_DIR2 17
#define MOTOR2_PIN_PWM 6
 
// PID params
double Kp = 0.4;  // Proportional coef
double Ki = 0.0;  // Іntegral coef
double Kd = 0.2;  // Diff coef
 
double integral = 0.0;
double previousError = 0.0;
 
TRSensors trs;
unsigned int sensorValues[NUM_SENSORS];
 

struct Motor {
  int pin_dir1;
  int pin_dir2;
  int pin_pwm;
};
 
Motor motor1 = {MOTOR1_PIN_DIR1, MOTOR1_PIN_DIR2, MOTOR1_PIN_PWM};
Motor motor2 = {MOTOR2_PIN_DIR1, MOTOR2_PIN_DIR2, MOTOR2_PIN_PWM};
 
void set_direction(Motor &motor, bool forward, bool invert = false) {
  if (invert) {
    forward = !forward;
  }
  digitalWrite(motor.pin_dir1, forward ? LOW : HIGH);
  digitalWrite(motor.pin_dir2, forward ? HIGH : LOW);
}
 
void set_speed(Motor &motor, int speed) {
  speed = constrain(speed, 0, MAX_SPEED);
  analogWrite(motor.pin_pwm, speed);
}
 
void setup() {
  Serial.begin(9600);
  Serial.println("Initializing line follower...");
 
 
  pinMode(motor1.pin_dir1, OUTPUT);
  pinMode(motor1.pin_dir2, OUTPUT);
  pinMode(motor1.pin_pwm, OUTPUT);
 
  pinMode(motor2.pin_dir1, OUTPUT);
  pinMode(motor2.pin_dir2, OUTPUT);
  pinMode(motor2.pin_pwm, OUTPUT);
 
 
  Serial.println("Calibrating sensors...");
  for (int i = 0; i < 400; i++) {
    trs.calibrate();
    delay(25);
  }
  Serial.println("Calibration complete");
}
 
void loop() {
 
  unsigned int position = trs.readLine(sensorValues);
  int error = position - 2000; 
 
  // PID 
  integral += error;
  double derivative = error - previousError;
  double correction = Kp * error + Ki * integral + Kd * derivative;
  previousError = error;
 
  
  int leftSpeed = DEFAULT_SPEED - correction;
  int rightSpeed = DEFAULT_SPEED + correction;
 
  
  leftSpeed = constrain(leftSpeed, 0, MAX_SPEED);
  rightSpeed = constrain(rightSpeed, 0, MAX_SPEED);
 
 
  set_direction(motor1, leftSpeed > 0); 
  set_speed(motor1, abs(leftSpeed));
 
  set_direction(motor2, rightSpeed > 0, true); 
  set_speed(motor2, abs(rightSpeed));}
 
