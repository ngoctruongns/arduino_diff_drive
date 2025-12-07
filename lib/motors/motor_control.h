#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "config.h"
#include "pid_controller.h"

// Motor define
typedef enum
{
    STOP = 0,
    FORWARD,
    BACKWARD
} MotorDirT;

typedef enum
{
    MOTOR_L = 0,
    MOTOR_R,
    MOTOR_MAX
} MotorNameT;

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
    void setPwm(uint8_t pwmVal);    // Only set PWM, keep direction
    void setPwmDir(uint8_t pwmVal, uint8_t dir);
    uint8_t getPwm() const;

    // PID methods
    void configPID(double Kp, double Ki, double Kd);
    void setLimitsOutput(double minVal, double maxVal);
    void enablePID(bool enable);
    void setTargetSpeed(double speed);
    void updateMeasuredSpeed(double speed);
    void computePID();

    double getTargetSpeed() const { return targetSpeed; }
    double getMeasuredSpeed() const { return measuredSpeed; }
    boolean isPIDEnabled() const { return pidEnabled; }

private:
    void setPwmDirect(uint8_t speed);
    void setDirDirect(bool in1, bool in2, bool flip);
};

/* Interface Functions Prototype */
void initMotor(void);
void stopMotor(void);
void setPwmMotor(MotorNameT name, uint8_t pwm, uint8_t dir);

void setPidControlEnabled(boolean enabled);
void updatePidParamsMotor(MotorNameT name, double Kp, double Ki, double Kd);
void setTargetSpeedMotor(MotorNameT name, double targetSpeed);
void controlMotorPIDLoop(void);

#endif // MOTOR_CONTROL_H