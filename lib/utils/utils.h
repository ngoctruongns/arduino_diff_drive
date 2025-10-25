/* Header for the utils as buzzer, led, etc. */

#ifndef UTILS_H
#define UTILS_H

#include "config.h"

/* Define buzzer state */
#define ON  HIGH
#define OFF LOW

/* Define color */
#define GREEN   0x00FF00
#define RED     0xFF0000
#define BLUE    0x0000FF
#define YELLOW  0xFFFF00
#define PURPLE  0xFF00FF
#define CYAN    0x00FFFF
#define WHITE   0xFFFFFF
#define BLACK   0x000000

/* Function prototype */
void initUtils(void);
void loopUtilsControl(void);

void setBuzzer(UINT8 state);
void setBuzzerOnTime(UINT16 onTime);

void ledBlink(UINT32 color, UINT16 delay_ms);
void ledRainbow(UINT8 first_hue, int8_t reps, UINT8 saturation, UINT8 brightness);
void setLedRgb(UINT8 r, UINT8 g, UINT8 b);
void setLedColor(UINT32 color);
void setLedColorBlink(UINT32 color, bool ledBlink);

// Joytick function
void initJoystick(void);
void loopJoytick(void);
int  getJoySpeedL(void);
int  getJoySpeedR(void);

#endif // UTILS_H