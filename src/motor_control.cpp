#include "config.h"
#include "motor_control.h"
#include <PID_v1.h>

volatile long g_encLeftCount = 0;
volatile long g_encRightCount = 0;

// Constructer 2 motor
MotorControl leftMotor(M1_PWM_PIN, M1_IN1_PIN, M1_IN2_PIN, false);
MotorControl rightMotor(M2_PWM_PIN, M2_IN1_PIN, M2_IN2_PIN, false);
// Interrupt encoder
void EncoderLeftISR()
{
    if (digitalRead(M1_ENCB_PIN))
    {
        g_encLeftCount++;
    }
    else
    {
        g_encLeftCount--;
    }
}

void EncoderRightISR()
{
    if (digitalRead(M2_ENCB_PIN))
    {
        g_encRightCount++;
    }
    else
    {
        g_encRightCount--;
    }
}

void initMotor()
{
    // Config encoder
    pinMode(M1_ENCA_PIN, INPUT_PULLUP);
    pinMode(M1_ENCB_PIN, INPUT_PULLUP);
    pinMode(M2_ENCA_PIN, INPUT_PULLUP);
    pinMode(M2_ENCB_PIN, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(M1_ENCA_PIN), EncoderLeftISR, RISING);
    attachInterrupt(digitalPinToInterrupt(M2_ENCA_PIN), EncoderRightISR, RISING);

    // Config motor end dir
    pinMode(M1_PWM_PIN, OUTPUT);
    pinMode(M1_IN1_PIN, OUTPUT);
    pinMode(M1_IN2_PIN, OUTPUT);

    pinMode(M2_PWM_PIN, OUTPUT);
    pinMode(M2_IN1_PIN, OUTPUT);
    pinMode(M2_IN2_PIN, OUTPUT);

    leftMotor.stop();
    rightMotor.stop();
}

void controlMotor(uint8_t name, uint8_t pwm, uint8_t dir)
{
    if (name == MOTOR_L) {
        leftMotor.setPwm(pwm, dir);
    }
    if (name == MOTOR_R) {
        rightMotor.setPwm(pwm, dir);
    }
}

void stopMotor(void)
{
    leftMotor.stop();
    rightMotor.stop();
}

MotorControl::MotorControl(uint8_t ena, uint8_t in1, uint8_t in2, bool flip)
    : enaPin(ena), in1Pin(in1), in2Pin(in2), pinFlip(flip)
{
    pinMode(enaPin, OUTPUT);
    pinMode(in1Pin, OUTPUT);
    pinMode(in2Pin, OUTPUT);
    stop();

    currentDir = STOP;
    currentPwm = 0;

    // pidController = new PID(&pidInput, &pidOutput, &pidSetpoint, 1.0, 0.0, 0.0, DIRECT);
    // pidController->SetOutputLimits(0, 255);
    // pidController->SetMode(MANUAL);
}

MotorControl::~MotorControl() {
    // delete pidController;
}



void MotorControl::stop() {
    setDirDirect(LOW, LOW, false);
    setPwmDirect(0);
}

uint8_t MotorControl::getPwm() const {
    return currentPwm;
}

void MotorControl::setPwm(uint8_t pwmVal){
    setPwmDirect(pwmVal);
}

void MotorControl::setPwm(uint8_t pwmVal,  uint8_t dir){
    switch (dir)
    {
    case STOP:
        setDirDirect(LOW, LOW, false);
        break;
    case FORWARD:
        setDirDirect(LOW, HIGH, pinFlip);
        break;
    case BACKWARD:
        setDirDirect(HIGH, LOW, pinFlip);
        break;
    default:
        break;
    }
    setPwmDirect(pwmVal);
}

// Private methods
void MotorControl::setPwmDirect(uint8_t pwm) {
    currentPwm = 0;
    if (pwm >= PWM_STOP_THRESHOLD) {
        currentPwm = constrain(pwm, PWM_MIN, PWM_MAX);
    }
    analogWrite(enaPin, currentPwm);
}

void MotorControl::setDirDirect(bool in1, bool in2, bool flip) { digitalWrite(in1Pin, in1);
    digitalWrite(in2Pin, in2);
}