#include "config.h"
#include "motor_control.h"
#include <PID_v1.h>

volatile long g_encLeftCount = 0;
volatile long g_encRightCount = 0;

// thêm biến thời gian xung để có thể ước lượng tốc độ khi delta = 0
volatile unsigned long g_prevLeftPulseMicros = 0;
volatile unsigned long g_lastLeftPulseMicros = 0;
volatile unsigned long g_prevRightPulseMicros = 0;
volatile unsigned long g_lastRightPulseMicros = 0;

// lưu hướng xung gần nhất (1 hoặc -1) để biết dấu tốc độ khi dùng khoảng thời gian
volatile int8_t g_lastLeftDir = 0;
volatile int8_t g_lastRightDir = 0;

// Constructer 2 motor
MotorControl leftMotor(M1_PWM_PIN, M1_IN1_PIN, M1_IN2_PIN, false);
MotorControl rightMotor(M2_PWM_PIN, M2_IN1_PIN, M2_IN2_PIN, false);

MotorControl *ptrMotorArr[2] = { &leftMotor, &rightMotor };

// Static function prototypes
static void updateMotorSpeeds(void);

/********************** Interrupt Functions ************************/
// Interrupt encoder
void EncoderLeftISR()
{
    // lưu thời gian xung và hướng
    g_prevLeftPulseMicros = g_lastLeftPulseMicros;
    g_lastLeftPulseMicros = micros();

    if (digitalRead(M1_ENCB_PIN))
    {
        g_encLeftCount++;
        g_lastLeftDir = 1;
    }
    else
    {
        g_encLeftCount--;
        g_lastLeftDir = -1;
    }
}

void EncoderRightISR()
{
    g_prevRightPulseMicros = g_lastRightPulseMicros;
    g_lastRightPulseMicros = micros();

    if (digitalRead(M2_ENCB_PIN))
    {
        g_encRightCount++;
        g_lastRightDir = 1;
    }
    else
    {
        g_encRightCount--;
        g_lastRightDir = -1;
    }
}

/**************************  Interface Functions  ***********************/
/************************************************************************/
void initMotor(void)
{
    // Configure PID parameters (adjust Kp, Ki, Kd values)
    leftMotor.configPID(KP_DEFAULT_LEFT, KI_DEFAULT_LEFT, KD_DEFAULT_LEFT);
    rightMotor.configPID(KP_DEFAULT_RIGHT, KI_DEFAULT_RIGHT, KD_DEFAULT_RIGHT);
    leftMotor.setLimitsOutput(-PWM_MAX, PWM_MAX);
    rightMotor.setLimitsOutput(-PWM_MAX, PWM_MAX);

    // Config encoder
    pinMode(M1_ENCA_PIN, INPUT_PULLUP);
    pinMode(M1_ENCB_PIN, INPUT_PULLUP);
    pinMode(M2_ENCA_PIN, INPUT_PULLUP);
    pinMode(M2_ENCB_PIN, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(M1_ENCA_PIN), EncoderLeftISR, RISING);
    attachInterrupt(digitalPinToInterrupt(M2_ENCA_PIN), EncoderRightISR, RISING);
}

void stopMotor(void)
{
    leftMotor.stop();
    rightMotor.stop();
}

void setPidControlEnabled(boolean enabled)
{
    leftMotor.enablePID(enabled);
    rightMotor.enablePID(enabled);
}

void setPwmMotor(MotorNameT name, uint8_t pwm, uint8_t dir)
{
    ptrMotorArr[name]->setPwmDir(pwm, dir);
}

void setTargetSpeedMotor(MotorNameT name, double targetSpeed)
{
    ptrMotorArr[name]->setTargetSpeed(targetSpeed);
}

void updatePidParamsMotor(MotorNameT name, double Kp, double Ki, double Kd)
{
    ptrMotorArr[name]->configPID(Kp, Ki, Kd);
}

void controlMotorPIDLoop(void)
{
    // Update motor speed from encoder readings
    updateMotorSpeeds();
    leftMotor.computePID();
    rightMotor.computePID();
}

/********************** Static functions **********************************/
/**************************************************************************/

// Tính tốc độ từ deltaCount trong dtSeconds; nếu deltaCount == 0 thì fallback sử dụng
// khoảng thời gian giữa 2 xung (prevPulseMicros -> lastPulseMicros).
static double calcSpeedFromEncoder(long deltaCount, double dtSeconds,
                                   unsigned long prevPulseMicros, unsigned long lastPulseMicros,
                                   int8_t lastDir)
{
    double rpm = 0.0;

    // Trường hợp có thay đổi số đếm trong dt: tốc độ = (deltaCount / PPR) / dt  [rev/s]*60 -> RPM
    if (dtSeconds > 0.0 && deltaCount != 0) {
        double revolutions = (double)deltaCount / (double)ENCODER_PPR;
        double revPerSec = revolutions / dtSeconds;
        rpm = revPerSec * 60.0;
    } else {
        // Fallback: nếu không có delta trong khoảng dt, dùng khoảng thời gian giữa 2 xung
        // interval = last - prev (microseconds) -> thời gian giữa 2 xung (1 count)
        if (prevPulseMicros != 0 && lastPulseMicros != 0 && lastPulseMicros > prevPulseMicros) {
            double intervalSec = (double)(lastPulseMicros - prevPulseMicros) / 1e6;
            if (intervalSec > 0.0) {
                // mỗi xung tương ứng 1/ENCODER_PPR vòng => rev/sec = 1 / (intervalSec * ENCODER_PPR)
                double revPerSec = 1.0 / (intervalSec * (double)ENCODER_PPR);
                rpm = revPerSec * 60.0;
            }
        } else {
            rpm = 0.0;
        }
    }

    // Áp dấu theo hướng xung gần nhất
    if (lastDir < 0) rpm = -fabs(rpm);
    return rpm;
}

// Cập nhật tốc độ động cơ bằng cách lấy delta count trong khoảng thời gian thực
static void updateMotorSpeeds(void)
{
    static long prevLeftCount = 0;
    static long prevRightCount = 0;
    static unsigned long prevTimeMicros = 0;

    // Lấy snapshot các biến volatile một cách nguyên tử
    noInterrupts();
    long curLeft = g_encLeftCount;
    long curRight = g_encRightCount;
    unsigned long prevLeftPulse = g_prevLeftPulseMicros;
    unsigned long lastLeftPulse = g_lastLeftPulseMicros;
    int8_t lastLeftDir = g_lastLeftDir;
    unsigned long prevRightPulse = g_prevRightPulseMicros;
    unsigned long lastRightPulse = g_lastRightPulseMicros;
    int8_t lastRightDir = g_lastRightDir;
    unsigned long nowMicros = micros();
    interrupts();

    double dtSeconds = 0.0;
    if (prevTimeMicros != 0 && nowMicros > prevTimeMicros) {
        dtSeconds = (double)(nowMicros - prevTimeMicros) / 1e6;
    }

    long deltaLeft = curLeft - prevLeftCount;
    long deltaRight = curRight - prevRightCount;

    double leftSpeed = calcSpeedFromEncoder(deltaLeft, dtSeconds, prevLeftPulse, lastLeftPulse, lastLeftDir);
    double rightSpeed = calcSpeedFromEncoder(deltaRight, dtSeconds, prevRightPulse, lastRightPulse, lastRightDir);

    leftMotor.updateMeasuredSpeed(leftSpeed);
    rightMotor.updateMeasuredSpeed(rightSpeed);

    // Lưu mẫu cho lần tính tiếp theo
    prevLeftCount = curLeft;
    prevRightCount = curRight;
    prevTimeMicros = nowMicros;
}

/********************  Motor control class functions  ******************/
/***********************************************************************/

MotorControl::MotorControl(uint8_t ena, uint8_t in1, uint8_t in2, bool flip)
    : enaPin(ena), in1Pin(in1), in2Pin(in2), pinFlip(flip),
      currentPwm(0), currentDir(STOP), targetSpeed(0),
      measuredSpeed(0), pidEnabled(false)
{
    pidController = new PIDController(); // Initial PID values

    pinMode(enaPin, OUTPUT);
    pinMode(in1Pin, OUTPUT);
    pinMode(in2Pin, OUTPUT);
}

MotorControl::~MotorControl()
{
    if (pidController) {
        delete pidController;
    }
}

void MotorControl::stop()
{
    setPwmDirect(0);
    currentDir = STOP;
}

void MotorControl::setPwm(uint8_t pwmVal)
{
    // Constrain pwmVal
    currentPwm = 0;
    if (pwmVal > PWM_STOP_THRESHOLD) {
        currentPwm = constrain(pwmVal, PWM_MIN, PWM_MAX);
    }
    setPwmDirect(currentPwm);
}

void MotorControl::setPwmDir(uint8_t pwmVal, uint8_t dir)
{
    setPwm(pwmVal);

    // Set direction
    if (dir == FORWARD) {
        setDirDirect(false, true, pinFlip);
    } else if (dir == BACKWARD) {
        setDirDirect(true, false, pinFlip);
    }
    currentDir = dir;
}

uint8_t MotorControl::getPwm() const
{
    return currentPwm;
}

void MotorControl::setPwmDirect(uint8_t speed)
{
    currentPwm = speed;
    analogWrite(enaPin, speed);
}

void MotorControl::setDirDirect(bool in1, bool in2, bool flip)
{
    if (flip) {
        in1 = !in1;
        in2 = !in2;
    }
    digitalWrite(in1Pin, in1);
    digitalWrite(in2Pin, in2);
}

/*************************  PID control functions  *********************/

// Set PID parameters
void MotorControl::configPID(double Kp, double Ki, double Kd)
{
    if (pidController) {
        pidController->setPidParams(Kp, Ki, Kd);
    }
}

void MotorControl::enablePID(bool enable)
{
    pidEnabled = enable;
    if (!enable) {
        pidController->reset();
    }
}

void MotorControl::setTargetSpeed(double speed)
{
    targetSpeed = speed;
    if (pidController) {
        pidController->setSetpoint(speed);
    }
}

void MotorControl::updateMeasuredSpeed(double speed)
{
    measuredSpeed = speed;
    if (pidController) {
        pidController->setInput(speed);
    }
}

void MotorControl::setLimitsOutput(double minVal, double maxVal)
{
    if (pidController) {
        pidController->setOutputLimits(minVal, maxVal);
    }
}

void MotorControl::computePID()
{
    if (pidEnabled && pidController) {
        double pwmValue = pidController->compute();
        // Get PWM value and direction
        uint8_t pwm = (uint8_t) fabs(pwmValue);
        uint8_t dir = (pwmValue >= 0) ? FORWARD : BACKWARD;
        setPwmDir(pwm, dir);
    }
}
