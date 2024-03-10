#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <AFMotor.h>

class MotorControl {
    public:
        MotorControl(int frontBackLeftWheelPin, int frontBackRightWheelPin);

        // Basic Movements
        void forward();
        void backward();
        void turnLeft();
        void turnRight();
        void stop();

    private:
        void setAllWheelsSpeed(int speed);

        AF_DCMotor frontBackLeftWheel;
        AF_DCMotor frontBackRightWheel;
};

#endif 