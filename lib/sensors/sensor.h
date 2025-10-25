// Code for HC-SR04 sensor: This project uses the HC-SR04 sensor to measure the distance.

#ifndef SENSOR_H
#define SENSOR_H

void    initSensor(void);

#if defined(USE_ULTRASONIC_SENSOR)
int     ultraGetDistance(void);
#endif // USE_ULTRASONIC_SENSOR

#endif // SENSOR_H