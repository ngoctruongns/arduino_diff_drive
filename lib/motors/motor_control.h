
#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "config.h"

// Motor define
enum
{
    STOP = 0,
    FORWARD,
    BACKWARD
};

enum
{
    MOTOR_L = 0,
    MOTOR_R
};

// Class
class MotorControl
{
private:
    // Motor pin control
    uint8_t enaPin;         // PWM pin
    uint8_t in1Pin;         // Pin1 for direction
    uint8_t in2Pin;         // Pin2 for direction
    boolean pinFlip;        // Select flip direction
    uint8_t currentPwm;     // 0 -> PWM_MAX
    uint8_t currentDir;     // Stop, Forward, Backward


public:
    MotorControl(uint8_t ena, uint8_t in1, uint8_t in2, bool flip);
    ~MotorControl();

    void stop();
    void setPwm(uint8_t pwmVal);
    void setPwm(uint8_t pwmVal, uint8_t dir);

    uint8_t getPwm() const;       // get current PWM value

private:
    void setPwmDirect(uint8_t speed);
    void setDirDirect(bool in1, bool in2, bool flip);


};

/* Function prototype */

void initMotor(void);
void controlMotor(uint8_t name, uint8_t pwm, uint8_t dir);
void stopMotor(void);

#endif // MOTOR_CONTROL_H

// class MotorControl1 {
//     private:
//         uint8_t enaPin;
//         uint8_t in1Pin;
//         uint8_t in2Pin;

//         PID* pidController;
//         double pidInput;
//         double pidOutput;
//         double pidSetpoint;

//         int currentPwm;
//         bool pidEnabled;

//     public:

//         MotorControl(uint8_t ena, uint8_t in1, uint8_t in2);
//         ~MotorControl();

//         void configPID(double Kp, double Ki, double Kd);
//         void enablePID();
//         void disablePID();

//         void updateSpeed(float measuredSpeed);
//         void runPID();

//         void forward(int speed);
//         void backward(int speed);
//         void stop();

//         void setTargetSpeed(float targetSpeed);
//         int getCurrentPWM() const;
//         double getTargetSpeed() const;
//         double getCurrentSpeed() const;

//     private:
//         void setSpeedDirect(int speed);
//         void setDirection(bool in1, bool in2);
//     };