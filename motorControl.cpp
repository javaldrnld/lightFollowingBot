#include <AFMotor.h>
#include "MotorControl.h"
#include <Arduino.h>

// Constant Variable
const int SPEED = 125;

MotorControl::MotorControl(int frontBackLeftWheelPin, int frontBackRightWheelPin) 
    : frontBackLeftWheel(frontBackLeftWheelPin), frontBackRightWheel(frontBackRightWheelPin) {
    frontBackLeftWheel.run(RELEASE);
    frontBackRightWheel.run(RELEASE);
}

void MotorControl::setAllWheelsSpeed(int speed) {
    frontBackLeftWheel.setSpeed(speed);
    frontBackRightWheel.setSpeed(speed);
}

void MotorControl::forward() {
    setAllWheelsSpeed(SPEED);

    frontBackLeftWheel.run(FORWARD);
    frontBackRightWheel.run(FORWARD);
}

void MotorControl::backward() {
    setAllWheelsSpeed(SPEED);

    frontBackLeftWheel.run(BACKWARD);
    frontBackRightWheel.run(BACKWARD);
}

void MotorControl::turnLeft() {
    frontBackLeftWheel.setSpeed(100);
    frontBackRightWheel.setSpeed(100);

    frontBackLeftWheel.run(BACKWARD);
    frontBackRightWheel.run(FORWARD);
}

void MotorControl::turnRight() {
    frontBackLeftWheel.setSpeed(100); // 100
    frontBackRightWheel.setSpeed(100);

    frontBackLeftWheel.run(FORWARD);
    frontBackRightWheel.run(BACKWARD);
}

void MotorControl::stop() {
    frontBackLeftWheel.run(RELEASE);
    frontBackRightWheel.run(RELEASE);
}