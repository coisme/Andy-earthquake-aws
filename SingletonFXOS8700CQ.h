// Copyright (c) Microsoft. All rights reserved.
// Licensed under the MIT license. See LICENSE file at https://github.com/Azure/azure-iot-sdks/blob/master/LICENSE for full license information.

#include "mbed.h"
#include "FXOS8700CQ.h"

typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
} AXES;

typedef struct
{
    AXES accelerometer;
    AXES magnometer;
} READING;

class SingletonFXOS8700CQ
{
private:

    SingletonFXOS8700CQ(PinName sda = PTE25, PinName scl = PTE24, int addr = FXOS8700CQ_SLAVE_ADDR1);
    SingletonFXOS8700CQ(const SingletonFXOS8700CQ &);
    SingletonFXOS8700CQ& operator=(const SingletonFXOS8700CQ&);
    
    FXOS8700CQ fxos;
    InterruptIn fxos_int1; // unused, common with SW2 on FRDM-K64F
    InterruptIn fxos_int2; // should just be the Data-Ready interrupt
    bool fxos_int1_triggered;
    bool fxos_int2_triggered;

    static void trigger_fxos_int1(void)
    {
        SingletonFXOS8700CQ::getInstance().setInt1Triggered(true);
    }
    
    static void trigger_fxos_int2(void)
    {
        SingletonFXOS8700CQ::getInstance().setInt2Triggered(true);
        //us_ellapsed = t.read_us();
    }
    
public:

    static SingletonFXOS8700CQ& getInstance()
    {
        static SingletonFXOS8700CQ instance;
        
        return instance;
    }
    
    void enable() { fxos.enable(); }
    void disable() { fxos.disable(); }
    
    uint8_t getData(READING&);
    
    bool getInt1Triggered() { return fxos_int1_triggered; }
    void setInt1Triggered(bool value) { fxos_int1_triggered = value; }
    bool getInt2Triggered() { return fxos_int2_triggered; }
    void setInt2Triggered(bool value) { fxos_int2_triggered = value; }
    uint8_t getWhoAmI() { return fxos.get_whoami(); }
    uint8_t getStatus() { return fxos.status(); }
};