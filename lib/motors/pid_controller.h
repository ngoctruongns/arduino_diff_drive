#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <Arduino.h>

class PIDController
{
private:
    double kp, ki, kd;
    double setpoint;
    double input;
    double output;
    double prevError;
    double integral;
    unsigned long lastTime;
    double outMin, outMax;

public:
    PIDController(double Kp, double Ki, double Kd);
    ~PIDController();

    void setSetpoint(double target);
    void setInput(double value);
    double compute();
    void setOutputLimits(double minVal, double maxVal);
    void reset();

    double getSetpoint() const { return setpoint; }
    double getInput() const { return input; }
    double getOutput() const { return output; }
    double getError() const { return setpoint - input; }
};

#endif // PID_CONTROLLER_H