#ifndef SENSOR_H
#define SENSOR_H

#include "MotionSensor/helper_3dmath.h"
#include "i2c.h"

class Sensor
{
public:
    Sensor();
    int ms_open();
    std::vector<float> ms_update(const int);
    int ms_close();

    uint8_t GetGravity(VectorFloat *v, Quaternion *q);
    uint8_t GetYawPitchRoll(float *data, Quaternion *q, VectorFloat *gravity);
    uint8_t GetGyro(int32_t *data, const uint8_t* packet);
};
#endif
