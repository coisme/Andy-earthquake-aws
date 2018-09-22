// Copyright (c) Microsoft. All rights reserved.
// Licensed under the MIT license. See LICENSE file at https://github.com/Azure/azure-iot-sdks/blob/master/LICENSE for full license information.

#include "SingletonFXOS8700CQ.h"

SingletonFXOS8700CQ::SingletonFXOS8700CQ(PinName sda, PinName scl, int addr) :
    fxos(sda, scl, addr),
    fxos_int1(PTC6),
    fxos_int2(PTC13)
{
    fxos_int2.fall(&trigger_fxos_int2);
}

uint8_t SingletonFXOS8700CQ::getData(READING &reading)
{
    SRAWDATA accel_data;
    SRAWDATA magn_data;
    
    memset(&reading, 0, sizeof(reading));
    
    uint8_t rc = fxos.get_data(&accel_data, &magn_data);
    
    if (rc == 0)
    {
        reading.accelerometer.x = accel_data.x;
        reading.accelerometer.y = accel_data.y;
        reading.accelerometer.z = accel_data.z;
        reading.magnometer.x = magn_data.x;
        reading.magnometer.y = magn_data.y;
        reading.magnometer.z = magn_data.z;
    }
    
    return rc;
}