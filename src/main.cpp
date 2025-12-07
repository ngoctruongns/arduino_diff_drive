#include "motor_control.h"
#include "sensor.h"
#include "utils.h"
#include "velocity_control.h"
#include "process_data_packet.h"

// Global variable
static UINT32 lastPidSampleMs;
static UINT32 lastPidTimeOutMs;

int disableRun = 0;
int speedL = 0;      // toc do dong co trai
int speedR = 0;      // toc do dong co phai
int dirL = 0;      // toc do dong co trai
int dirR = 0;      // toc do dong co phai
int speedLevel = 0;
const int pwmStep = (PWM_MAX - PWM_MIN) / PWM_STEP_NUM;

boolean isPidControl = false;

uint8_t rx_buffer[BUFFER_SIZE] = {0};
uint8_t tx_buffer[BUFFER_SIZE] = {0};

// Send encoder data every 100ms
unsigned long last_send_time = 0;
const unsigned long send_interval = 500; // milliseconds

// Serial for bluetooth
HardwareSerial& bleSerial = BLE_SERIAL;

/******************  Function body ******************/

void encoderSend(int32_t left_enc, int32_t right_enc)
{
    WheelEncType enc_data;
    enc_data.type = WHEEL_ENC_COMMAND;
    enc_data.left_enc = left_enc;
    enc_data.right_enc = right_enc;
    uint8_t data_len = sizeof(WheelEncType);

    // Prepare data package
    uint8_t tx_len = encoderAllPackage((uint8_t *)&enc_data, data_len, tx_buffer);

    if (tx_len > 0) {
        // Send data via Serial
        Serial.write(tx_buffer, tx_len);
        Serial.println("Encoder send");
    }
}

void decoderCmdVel(uint8_t *buff, uint8_t len)
{
    CmdVelType cmd_vel;
    memcpy(&cmd_vel, buff, len);
    int left_rpm  = cmd_vel.left_rpm;
    int right_rpm = cmd_vel.right_rpm;

    // Print rpm value
    // printf("Received: L= %d, R= %d\n", left_rpm, right_rpm);
    Serial.print("L= ");
    Serial.print(left_rpm);
    Serial.print(", R= ");
    Serial.println(right_rpm);
}

/******************  Main program ******************/
void setup()
{
    Serial.begin(SERIAL_BAUDRATE);
    bleSerial.begin(BLE_BAUDRATE);
    initMotor();
    initSensor();
    initUtils();

    // update last times
    lastPidSampleMs = millis();
    lastPidTimeOutMs = millis();
}

void loop()
{
    loopUtilsControl();

#if defined(USE_ULTRASONIC_SENSOR)
    // Check distance sensor
    int distance = ultraGetDistance();
    if (distance < DISABLE_RUN_CM) {
        if (disableRun == 0) {
            disableRun = 1;
            setLedColorBlink(RED, ON);
            setBuzzerOnTime(100);
        }
        Serial.println(distance);
    } else {
        if (disableRun == 1) {
            disableRun = 0;
            setLedColorBlink(GREEN, OFF);
        }
    }

#endif // USE_ULTRASONIC_SENSOR

    // Priority for Joystick control over PID
    if (getIsJoystickControl()) {
        if (isPidControl) {
            isPidControl = false;
            setPidControlEnabled(false);
            Serial.println("Joystick control, stop PID");
        }

        int joySpeedL = getJoySpeedL();
        int joySpeedR = getJoySpeedR();

        if (joySpeedL > 0) {
            speedL = joySpeedL;
            dirL = FORWARD;
        } else {
            speedL = -joySpeedL;
            dirL = BACKWARD;
        }

        if (joySpeedR > 0) {
            speedR = joySpeedR;
            dirR = FORWARD;
        } else {
            speedR = -joySpeedR;
            dirR = BACKWARD;
        }

        digitalWrite(LED_BUILTIN, HIGH);
        Serial.print(joySpeedL);
        Serial.print(" <-> ");
        Serial.println(joySpeedR);

        if (disableRun == 0 || dirL == BACKWARD || dirR == BACKWARD) {
            setPwmMotor(MOTOR_L, speedL, dirL);
            setPwmMotor(MOTOR_R, speedR, dirR);
        }

        delay(20);
    } else {
        speedL = 0;
        speedR = 0;
        digitalWrite(LED_BUILTIN, LOW);

        if (isPidControl) {
            UINT32 currentMillis = millis();
            if (currentMillis - lastPidTimeOutMs >= PID_CONTROL_TIMEOUT) {
                lastPidTimeOutMs = currentMillis;
                // Nếu quá thời gian không nhận lệnh điều khiển, tắt PID
                isPidControl = false;
                setPidControlEnabled(false);
                Serial.println("PID control timeout, stop PID");
            }

            if (currentMillis - lastPidSampleMs >= PID_SAMPLE_TIME) {
                lastPidSampleMs = currentMillis;
                controlMotorPIDLoop();
            }
        }
    }
}
