#include "utils.h"
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
 #include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif


Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUMBER_LEDS, LED_RGB_PIN, NEO_GRB + NEO_KHZ800);

UINT8 buzzerState = OFF;
UINT16 buzzerOnTime = 0;
UINT32 previousBuzzerMs = 0;

UINT32 ledColor = GREEN;
bool ledState = OFF;
bool ledBlinkState = OFF;

/* Function prototype */
static void loopLedControl(void);
static void loopBuzzerControl(void);

/*************** Utils Function  *******************/
void initUtils(void)
{
    // LED strip Init
    strip.begin();
    strip.clear();
    strip.setBrightness(50);
    setLedColorBlink(GREEN, OFF);

    // Buzzer Init
    pinMode(BUZZER_PIN, OUTPUT);
    setBuzzer(ON); // bat coi
    delay(200);
    setBuzzer(OFF); // tat coi

    // Joystick Init
    initJoystick();
}

// Function to control utils loop control use millis
void loopUtilsControl(void)
{
    loopJoytick();
    loopLedControl();
    loopBuzzerControl();
}

/*************** Buzzer Function  *******************/

// Function to set buzzer state
void setBuzzer(UINT8 state)
{
    digitalWrite(BUZZER_PIN, state);
    buzzerState = state;
}

// Function to set buzzer on time
void setBuzzerOnTime(UINT16 onTime)
{
    buzzerOnTime = onTime;
    previousBuzzerMs = millis();
    setBuzzer(ON);
}

// Function to loop of buzzer control
static void loopBuzzerControl(void)
{
    unsigned long currentMillis = millis();

    if (buzzerOnTime > 0)
    {
        if (currentMillis - previousBuzzerMs >= buzzerOnTime)
        {
            previousBuzzerMs = currentMillis;
            buzzerOnTime = 0;
            setBuzzer(OFF);
        }
    } else {
        if (buzzerState) {
            setBuzzer(OFF);
        }
    }
}

/**************** LED Function  *********************/

// Function to set led color
void setLedRgb(UINT8 r, UINT8 g, UINT8 b)
{
    for (int i = 0; i < NUMBER_LEDS; i++)
    {
        strip.setPixelColor(i, strip.Color(r, g, b));
    }
    strip.show();
}

// Function to set led color use hex color
void setLedColor(UINT32 color)
{
    for (int i = 0; i < NUMBER_LEDS; i++)
    {
        strip.setPixelColor(i, color);
    }
    strip.show();
}

/* Led blink with delay */
void ledBlink(UINT32 color, UINT16 delay_ms)
{
    setLedColor(color);
    delay(delay_ms);
    setLedColor(BLACK);
    delay(delay_ms);
}

// function for strip led rainbow effect without delay (use millis instead)
void ledRainbow(UINT8 first_hue, int8_t reps, UINT8 saturation, UINT8 brightness)
{
    UINT16 i, j;

    for (j = 0; j < (UINT16) 256 * reps; j++)
    { // 5 cycles of all colors on wheel
        for (i = 0; i < strip.numPixels(); i++)
        {
            strip.setPixelColor(i, strip.ColorHSV(first_hue + (i * 256 / strip.numPixels()), saturation, brightness));
        }
        strip.show();
    }
}

// Function to loop of led control
static void loopLedControl(void)
{
    static unsigned long previousLedMs = 0;
    unsigned long currentMillis = millis();

    if (ledBlinkState)
    {
        if (currentMillis - previousLedMs >= LED_DELAY_VAL)
        {
            previousLedMs = currentMillis;
            ledState = !ledState;
            if (ledState)
            {
                setLedColor(ledColor);
            }
            else
            {
                setLedColor(BLACK);
            }
        }
    } else {
        setLedColor(ledColor);
    }
}

// Fucntion to set Led color
void setLedColorBlink(UINT32 color, bool ledBlink)
{
    ledColor = color;
    ledBlinkState = ledBlink;
    setLedColor(color);
}