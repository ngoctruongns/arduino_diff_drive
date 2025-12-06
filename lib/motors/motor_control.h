#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "config.h"
#include "pid_controller.h"

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

    // PID control
    PIDController* pidController;
    double targetSpeed;     // Target speed (e.g., RPM or m/s)
    double measuredSpeed;   // Measured speed from encoder
    boolean pidEnabled;

public:
    MotorControl(uint8_t ena, uint8_t in1, uint8_t in2, bool flip);
    ~MotorControl();

    void stop();
    void setPwm(uint8_t pwmVal);
    void setPwm(uint8_t pwmVal, uint8_t dir);

    // PID methods
    void configPID(double Kp, double Ki, double Kd);
    void enablePID(bool enable);
    void setTargetSpeed(double speed);
    void updateMeasuredSpeed(double speed);
    void computePID();

    uint8_t getPwm() const;
    double getTargetSpeed() const { return targetSpeed; }
    double getMeasuredSpeed() const { return measuredSpeed; }
    boolean isPIDEnabled() const { return pidEnabled; }

private:
    void setPwmDirect(uint8_t speed);
    void setDirDirect(bool in1, bool in2, bool flip);
};

/* Function prototype */

void initMotor(void);
void controlMotor(uint8_t name, uint8_t pwm, uint8_t dir);
void controlMotorPID(uint8_t name, double targetSpeed);
void stopMotor(void);
void updateMotorSpeeds(void);
void computeMotorPID(void);

#endif // MOTOR_CONTROL_H