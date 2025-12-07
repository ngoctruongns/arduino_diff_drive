#include "config.h"
#include "utils.h"
#include <PS2X_lib.h> //for v1.6

/******************************************************************
 * set pins connected to PS2 controller:
 *   - 1e column: original
 *   - 2e colmun: Stef?
 * replace pin numbers by the ones you use
 ******************************************************************/
#define PS2_SEL SS      // SS    -> pin 10 for Uno
#define PS2_DAT MOSI    // MOSI  -> pin 11 for Uno
#define PS2_CMD MISO    // MISO  -> pin 12 for Uno
#define PS2_CLK SCK     // SCK   -> pin 13 for Uno

/******************************************************************
 * select modes of PS2 controller:
 *   - pressures = analog reading of push-butttons
 *   - rumble    = motor rumbling
 * uncomment 1 of the lines for each mode selection
 ******************************************************************/
#define pressures true  // true = enable analog input, false = digital input
#define rumble false    // true = enable rumble, false = disable rumble

PS2X ps2x; // create PS2 Controller Class

// right now, the library does NOT support hot pluggable controllers, meaning
// you must always either restart your Arduino after you connect the controller,
// or call config_gamepad(pins) again after connecting the controller.

int error = 0;
byte type = 0;
byte vibrate = 0;

static int joySpeedL = 0;      // toc do dong co trai
static int joySpeedR = 0;      // toc do dong co phai
static boolean isJoystickControl = false;

// Hàm tính toán tốc độ động cơ
static void calcSpeed(uint8_t leftY, uint8_t rightX) {
    // Chuẩn hóa giá trị joystick về khoảng [-1, 1]
    float xNorm = (float)(JOYSTICK_MID_Y - leftY) / JOYSTICK_MAX;
    float yNorm = (float)(JOYSTICK_MID_X - rightX) / JOYSTICK_MAX;

    // Tính vận tốc thẳng và vận tốc góc
    float v = xNorm * PWM_MAX;              // Vận tốc thẳng
    float omega = yNorm * OMEGA_MAX;      // Vận tốc góc

    // Tính vận tốc cho từng động cơ
    float vL = v - (omega ) / 2.0; // Động cơ trái
    float vR = v + (omega ) / 2.0; // Động cơ phải

    // Chuyển đổi vận tốc sang giá trị PWM (0-255)
    joySpeedL = (int)vL;
    joySpeedR = (int)vR;

    // Set flag isJoystickControl if velocity is over threshold
    if (abs(joySpeedL) > JOYSTICK_DEADZONE || abs(joySpeedR) > JOYSTICK_DEADZONE) {
        isJoystickControl = true;
    } else {
        isJoystickControl = false;
    }
}

void initJoystick()
{
    delay(300); // added delay to give wireless ps2 module some time to startup, before configuring it

    // CHANGES for v1.6 HERE!!! **************PAY ATTENTION*************

    // setup pins and settings: GamePad(clock, command, attention, data, Pressures?, Rumble?) check for error
    for (int i =0; i<10; i++)
    {
        error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);
        if(error == 0)
        {
            break;
        }
    }


    if (error == 0)
    {
        Serial.print("Found Controller, configured successful ");
        Serial.print("pressures = ");
        if (pressures)
            Serial.println("true ");
        else
            Serial.println("false");
        Serial.print("rumble = ");
        if (rumble)
            Serial.println("true)");
        else
            Serial.println("false");
        // set buzzer to notify that controller was found
        setBuzzer(ON);
        delay(200);
        setBuzzer(OFF);
    }
    else if (error == 1)
        Serial.println("No controller found, check wiring, see readme.txt to enable debug. visit www.billporter.info for troubleshooting tips");

    else if (error == 2)
        Serial.println("Controller found but not accepting commands. see readme.txt to enable debug. Visit www.billporter.info for troubleshooting tips");

    else if (error == 3)
        Serial.println("Controller refusing to enter Pressures mode, may not support it. ");

    //  Serial.print(ps2x.Analog(1), HEX);

    type = ps2x.readType();
    switch (type)
    {
    case 0:
        Serial.print("Unknown Controller type found ");
        break;
    case 1:
        Serial.print("DualShock Controller found ");
        break;
    case 2:
        Serial.print("GuitarHero Controller found ");
        break;
    case 3:
        Serial.print("Wireless Sony DualShock Controller found ");
        break;
    }
}

void loopJoytick()
{
    /*  You must Read Gamepad to get new values and set vibration values
        ps2x.read_gamepad(small motor on/off, larger motor strenght from 0-255)
        if you don't enable the rumble, use ps2x.read_gamepad(); with no values
        You should call this at least once a second
     */
    isJoystickControl = false;
    if (error == 1) // skip loop if no controller found
        return;

    if (type == 2)
    {                        // Guitar Hero Controller
        ps2x.read_gamepad(); // read controller

        if (ps2x.ButtonPressed(GREEN_FRET))
            Serial.println("Green Fret Pressed");
        if (ps2x.ButtonPressed(RED_FRET))
            Serial.println("Red Fret Pressed");
        if (ps2x.ButtonPressed(YELLOW_FRET))
            Serial.println("Yellow Fret Pressed");
        if (ps2x.ButtonPressed(BLUE_FRET))
            Serial.println("Blue Fret Pressed");
        if (ps2x.ButtonPressed(ORANGE_FRET))
            Serial.println("Orange Fret Pressed");

        if (ps2x.ButtonPressed(STAR_POWER))
            Serial.println("Star Power Command");

        if (ps2x.Button(UP_STRUM)) // will be TRUE as long as button is pressed
            Serial.println("Up Strum");
        if (ps2x.Button(DOWN_STRUM))
            Serial.println("DOWN Strum");

        if (ps2x.Button(PSB_START)) // will be TRUE as long as button is pressed
            Serial.println("Start is being held");
        if (ps2x.Button(PSB_SELECT))
            Serial.println("Select is being held");

        if (ps2x.Button(ORANGE_FRET))
        { // print stick value IF TRUE
            Serial.print("Wammy Bar Position:");
            Serial.println(ps2x.Analog(WHAMMY_BAR), DEC);
        }
    }
    else
    {                                      // DualShock Controller
        ps2x.read_gamepad(false, vibrate); // read controller and set large motor to spin at 'vibrate' speed

        if (ps2x.Button(PSB_START)) // will be TRUE as long as button is pressed
            Serial.println("Start is being held");
        if (ps2x.Button(PSB_SELECT))
            Serial.println("Select is being held");

        if (ps2x.Button(PSB_PAD_UP))
        { // will be TRUE as long as button is pressed
            Serial.print("Up held this hard: ");
            Serial.println(ps2x.Analog(PSAB_PAD_UP), DEC);
        }
        if (ps2x.Button(PSB_PAD_RIGHT))
        {
            Serial.print("Right held this hard: ");
            Serial.println(ps2x.Analog(PSAB_PAD_RIGHT), DEC);
        }
        if (ps2x.Button(PSB_PAD_LEFT))
        {
            Serial.print("LEFT held this hard: ");
            Serial.println(ps2x.Analog(PSAB_PAD_LEFT), DEC);
        }
        if (ps2x.Button(PSB_PAD_DOWN))
        {
            Serial.print("DOWN held this hard: ");
            Serial.println(ps2x.Analog(PSAB_PAD_DOWN), DEC);
        }

        vibrate = ps2x.Analog(PSAB_CROSS); // this will set the large motor vibrate speed based on how hard you press the blue (X) button
        if (ps2x.NewButtonState())
        { // will be TRUE if any button changes state (on to off, or off to on)
            if (ps2x.Button(PSB_L3))
                Serial.println("L3 pressed");
            if (ps2x.Button(PSB_R3))
                Serial.println("R3 pressed");
            if (ps2x.Button(PSB_L2))
                Serial.println("L2 pressed");
            if (ps2x.Button(PSB_R2))
                Serial.println("R2 pressed");
            if (ps2x.Button(PSB_TRIANGLE)) {
                setBuzzerOnTime(200);
                Serial.println("Triangle pressed");
            }
        }

        if (ps2x.ButtonPressed(PSB_CIRCLE)) // will be TRUE if button was JUST pressed
            Serial.println("Circle just pressed");
        if (ps2x.NewButtonState(PSB_CROSS)) // will be TRUE if button was JUST pressed OR released
            Serial.println("X just changed");
        if (ps2x.ButtonReleased(PSB_SQUARE)) // will be TRUE if button was JUST released
            Serial.println("Square just released");

        joySpeedL = joySpeedR = 0;
        if (ps2x.Button(PSB_L1) || ps2x.Button(PSB_R1))
        { // print stick values if either is TRUE
            // Serial.print("PSS_LY:");
            // Serial.print(ps2x.Analog(PSS_LY), DEC); // Left stick, Y axis. Other options: LX, RY, RX
            // Serial.print(",");
            // Serial.print("PSS_RX:");
            // Serial.println(ps2x.Analog(PSS_RX), DEC);

            calcSpeed(ps2x.Analog(PSS_LY), ps2x.Analog(PSS_RX));
        }
    }
}

int getJoySpeedL()
{
    return joySpeedL;
}

int getJoySpeedR()
{
    return joySpeedR;
}

boolean getIsJoystickControl()
{
    return isJoystickControl;
}
