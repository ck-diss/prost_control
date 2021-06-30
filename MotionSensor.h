#ifndef _MOTION_SENSOR_H_
#define _MOTION_SENSOR_H_

#define YAW 0
#define PITCH 1
#define ROLL 2
#define DIM 3

#include <QDebug>


#include "MotionSensor/helper_3dmath.h"

extern float ypr[3]; //yaw, pitch, roll
extern float accel[3];
extern float gyro[3];
extern float temp;
extern float compass[3];

extern int ms_open();
extern std::vector<float> ms_update(const int);
extern int ms_close();

#endif
