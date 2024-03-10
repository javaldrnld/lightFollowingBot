#include "MotorControl.h"
#include <NewPing.h>
#include "LightDetector.h"
#include <Servo.h>

// DEFINE ULTRASONIC PINS
#define TRIG_PIN A2
#define ECHO_PIN A3
#define MAX_DISTANCE 100

// DEFINE SERVO PIN
#define SERVO_PIN A1

// DEFINE WHEELS
#define FRONT_BACK_LEFT_WHEEL 2 // Pakibago nalang
#define FRONT_BACK_RIGHT_WHEEL 3 // Pakibago nalang

// DEFINE LDR PINS -> Set as a digital?
#define LEFT_LDR_PIN A0 // Pakibago nalang
#define RIGHT_LDR_PIN A4 // Pakibago nalang


// Initialize Objects
MotorControl motorWheel(FRONT_BACK_LEFT_WHEEL, FRONT_BACK_RIGHT_WHEEL);
NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_DISTANCE);
Servo myservo;
LightDetector leftSensor(LEFT_LDR_PIN);
LightDetector rightSensor(RIGHT_LDR_PIN);

volatile bool obstacleDetected = false; // Delete


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  myservo.attach(SERVO_PIN);
  attachInterrupt(digitalPinToInterrupt(ECHO_PIN), obstacleISR, CHANGE);

}

void loop() {
  // put your main code here, to run repeatedly:
  int leftLDR = leftSensor.read();
  int rightLDR = rightSensor.read();
  unsigned int distance = sonar.ping_cm();
  // motorWheel.forward();

  // Logic 
  // if (distance > 25) {
  //   myservo.write(90);
  //   motorWheel.forward();
  // } else 
  if (distance < 10 && distance > 0) {
    Serial.println("Avoiding");
    avoidObstacle();
    obstacleDetected = false;
  } else if (leftLDR == 0 && rightLDR == 0) {
    Serial.println("Forward");
    motorWheel.forward();  
  } else if (leftLDR == 0 && rightLDR == 1) {
    Serial.println("Right");
    motorWheel.turnRight();
  } else if (leftLDR == 1 && rightLDR == 0) {
    Serial.println("LEft");
    motorWheel.turnLeft();
  } else if (leftLDR == 1 && rightLDR == 1) {
    motorWheel.stop();
  }
  delay(100);
}

void avoidObstacle() {
  // put your main code here, to run repeatedly:
  motorWheel.stop();
  delay(100);

  myservo.write(0); // Look for right obstacle
  delay(500); // Let it finish

  // Calculate the distance again by newping
  unsigned int distance = sonar.ping_cm();
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");
  delay(10); // Let it finish reading

  if (distance > 25) { // If the distance is greater than 25cm
    // Rotate the servo to center angle
    myservo.write(90);
    delay(500);

    // Move backward
    motorWheel.backward();
    delay(100);
    motorWheel.stop();
    delay(100);

    //Turn left
    motorWheel.turnLeft();
    delay(500);
  } else if ((distance < 20) && (distance > 0)) {
    // Rotate the servo to 180 degree (left)
    Serial.println("In the else if");
    myservo.write(180);
    delay(500);

    distance = sonar.ping_cm();
    delay(10);

    if (distance > 25) {
      // Rotate the serve to center angle
      myservo.write(90);
      delay(500);

      motorWheel.backward();
      delay(100);
      motorWheel.stop();
      delay(100);
      motorWheel.turnRight();
      delay(500);
    } 
  }
}

void obstacleISR() {
  if (sonar.ping_cm() < 10) {
    obstacleDetected = true;
  }
}