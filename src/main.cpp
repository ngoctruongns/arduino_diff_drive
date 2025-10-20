#include "motor_control.h"
#include "sensor.h"
#include "utils.h"

// Global variable
UINT8 ctlCommand;
UINT32 previousMillis;

int disableRun = 0;
int speedL = 0;      // toc do dong co trai
int speedR = 0;      // toc do dong co phai
int dirL = 0;      // toc do dong co trai
int dirR = 0;      // toc do dong co phai
int speedLevel = 0;
const int pwmStep = (PWM_MAX - PWM_MIN) / PWM_STEP_NUM;

boolean joystickControl = false;

// Serial for bluetooth
HardwareSerial& bleSerial = BLE_SERIAL;

void setup()
{
    Serial.begin(9600);
    bleSerial.begin(9600);
    initMotor();
    initSensor();
    initUtils();
}

void loop()
{
    loopUtilsControl();
    // Check distance sensor
    int distance = ultraGetDistance();
    if (distance < DISABLE_RUN_CM)
    {
        if (disableRun == 0)
        {
            disableRun = 1;
            setLedColorBlink(RED, ON);
            setBuzzerOnTime(100);
        }
        Serial.println(distance);
    }
    else
    {
        if (disableRun == 1)
        {
            disableRun = 0;
            setLedColorBlink(GREEN, OFF);
        }
    }

    // get speed from joystick
    int joySpeedL = getJoySpeedL();
    int joySpeedR = getJoySpeedR();
    if (joySpeedL != 0 || joySpeedR != 0)
    {
        previousMillis = millis();
        joystickControl = true;
        joySpeedL = constrain(joySpeedL, -PWM_MAX, PWM_MAX);
        joySpeedR = constrain(joySpeedR, -PWM_MAX, PWM_MAX);
        if ( joySpeedL > 0) {
            speedL = joySpeedL;
            dirL = FORWARD;
        } else {
            speedL = - joySpeedL;
            dirL = BACKWARD;
        }

        if ( joySpeedR > 0) {
            speedR = joySpeedR;
            dirR = FORWARD;
        } else {
            speedR = - joySpeedR;
            dirR = BACKWARD;
        }
        Serial.print(joySpeedL);
        Serial.print(" <-> ");
        Serial.println(joySpeedR);
    } else {
        if (joystickControl) {
            joystickControl = false;
            speedL = 0;
            speedR = 0;
        }
    }

    // check if data has been sent from the computer:
    if (bleSerial.available())
    {
        // read the most recent byte (which will be from 0 to PWM_MAX):
        ctlCommand = bleSerial.read();   // doc gia tri nhan
        previousMillis = millis(); // dem thoi gian luc hien tai
        bleSerial.print("OK");        //  Gửi xác nhận đã nhận được dữ liệu
    }

    // xu ly khi co coi
    if (ctlCommand == 'C')
    {
        setBuzzerOnTime(200); // bat coi
    }
    if (ctlCommand == 'D')
    {

        setBuzzer(LOW); // tat coi
    }

    // Xe chay tien
    if ((ctlCommand == 'T') && (disableRun == 0))
    {
        // Serial.println("Tien");
        digitalWrite(LED_BUILTIN, HIGH);
        speedLevel = bleSerial.parseInt();
        speedL = PWM_MIN + speedLevel * pwmStep;
        speedR = PWM_MIN + speedLevel * pwmStep;
        dirL = FORWARD;
        dirR = FORWARD;
    }

    if ((ctlCommand == 'L') && (disableRun == 0))
    {
        // Serial.println("Trai");
        digitalWrite(LED_BUILTIN, HIGH);
        speedLevel = bleSerial.parseInt();
        speedL = 0;
        speedR = PWM_MIN + speedLevel * pwmStep;
        dirL = FORWARD;
        dirR = FORWARD;
    }
    if ((ctlCommand == 'R') && (disableRun == 0))
    {
        // Serial.println("Phai");
        digitalWrite(LED_BUILTIN, HIGH);
        speedLevel = bleSerial.parseInt();
        speedL = PWM_MIN + speedLevel * pwmStep;
        speedR = 0;
        dirL = FORWARD;
        dirR = FORWARD;
    }
    if (ctlCommand == 'B')
    {
        // Serial.println("Lui");
        digitalWrite(LED_BUILTIN, HIGH);
        speedLevel = bleSerial.parseInt();
        speedL = PWM_MIN + speedLevel * pwmStep;
        speedR = PWM_MIN + speedLevel * pwmStep;
        dirL = BACKWARD;
        dirR = BACKWARD;
    }



    if (disableRun == 0 || (dirL == BACKWARD && dirR == BACKWARD)) {
        controlMotor(MOTOR_L, speedL, dirL);
        controlMotor(MOTOR_R, speedR, dirR);
    }

    if ((millis() - previousMillis) >= CMD_TIME_OVER) {
        ctlCommand = 'S';         // xoa du lieu
    }

    if (ctlCommand == 'S' && joystickControl == false) {
        //bleSerial.println("Dung");
        digitalWrite(LED_BUILTIN, LOW);
        stopMotor();
    }

    delay(20);
}