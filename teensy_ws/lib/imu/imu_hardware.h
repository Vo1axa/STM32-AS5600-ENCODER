// Copyright (c) 2021 Juan Miguel Jimeno
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef DEFAULT_IMU
#define DEFAULT_IMU

#include "imu_interface.h"

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

class BNO055_IMU : public IMUInterface
{
    Adafruit_BNO055 bno;
    uint16_t BNO055_SAMPLERATE_DELAY_MS = 10;
    double ACCEL_VEL_TRANSITION = (double)(BNO055_SAMPLERATE_DELAY_MS) / 1000.0;
    double ACCEL_POS_TRANSITION = 0.5 * ACCEL_VEL_TRANSITION * ACCEL_VEL_TRANSITION;

public:
    BNO055_IMU() : bno(Adafruit_BNO055(55, 0x28, &Wire1)) {}

    bool startSensor() override
    {
        if (!bno.begin())
        {
            return false;
        }
        delay(1000);
        bno.setExtCrystalUse(true);

        return true;
    }
    geometry_msgs__msg__Vector3 readAccelerometer() override
    {
        sensors_event_t event;
        bno.getEvent(&event, Adafruit_BNO055::VECTOR_LINEARACCEL);
        geometry_msgs__msg__Vector3 accel;
        accel.x = event.acceleration.x; // Konversi ke m/s²
        accel.y = event.acceleration.y;
        accel.z = event.acceleration.z;
        return accel;
    }
    geometry_msgs__msg__Vector3 readGyroscope() override
    {
        sensors_event_t event;
        bno.getEvent(&event, Adafruit_BNO055::VECTOR_GYROSCOPE);

        geometry_msgs__msg__Vector3 gyro;
        gyro.x = event.gyro.x; // Data sudah dalam rad/s
        gyro.y = event.gyro.y;
        gyro.z = event.gyro.z * -1;

        return gyro;
    }
    // geometry_msgs__msg__Vector3 readOrientation() override
    // {
    //     sensors_event_t event;
    //     bno.getEvent(&event, Adafruit_BNO055::VECTOR_EULER);

    //     geometry_msgs__msg__Vector3 orient;

    //     orient.x = event.orientation.x;
    //     orient.y = event.orientation.y;
    //     orient.z = event.orientation.z;

    //     return orient;
    // }
};
#endif