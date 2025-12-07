#include "pid_controller.h"

PIDController::PIDController()
    : kp(1.0), ki(0), kd(0), setpoint(0), input(0), output(0),
      prevError(0), integral(0), lastTime(0), outMin(-255), outMax(255)
{
}

PIDController::~PIDController()
{
}

void PIDController::setPidParams(double Kp, double Ki, double Kd)
{
    kp = Kp;
    ki = Ki;
    kd = Kd;
}

void PIDController::setSetpoint(double target)
{
    setpoint = target;
}

void PIDController::setInput(double value)
{
    input = value;
}

double PIDController::compute()
{
    unsigned long currentTime = millis();
    double timeChange = (currentTime - lastTime) / 1000.0; // Convert to seconds

    if (lastTime == 0) {
        lastTime = currentTime;
        return output;
    }

    double error = setpoint - input;

    // Proportional term
    double pTerm = kp * error;

    // Integral term
    integral += error * timeChange;
    double iTerm = ki * integral;

    // Derivative term
    double dTerm = kd * (error - prevError) / timeChange;

    // Calculate output
    output = pTerm + iTerm + dTerm;

    // Limit output
    if (output > outMax) output = outMax;
    if (output < outMin) output = outMin;

    // Save for next iteration
    prevError = error;
    lastTime = currentTime;

    return output;
}

void PIDController::setOutputLimits(double minVal, double maxVal)
{
    outMin = minVal;
    outMax = maxVal;
}

void PIDController::reset()
{
    prevError = 0;
    integral = 0;
    lastTime = 0;
    output = 0;
}