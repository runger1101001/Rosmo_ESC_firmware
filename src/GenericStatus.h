#ifndef __GENERIC_STATUS_H__
#define __GENERIC_STATUS_H__

#include <inttypes.h>


enum SimpleFOCStatusIndication : uint8_t {
    SIMPLEFOC_STATUS_OFF =     0x00,
    SIMPLEFOC_STATUS_ON =      0x01,
    SIMPLEFOC_INITIALIZING =   0x02,
    SIMPLEFOC_CALIBRATING =    0x03,

    SIMPLEFOC_FAULT =          0x80,
    SIMPLEFOC_MOTOR_FAULT =    0x81,
    SIMPLEFOC_SENSOR_FAULT =   0x82,
    SIMPLEFOC_COMM_FAULT =     0x83,
    SIMPLEFOC_TEMP_FAULT =     0x84,
    SIMPLEFOC_VBUSHIGH_FAULT = 0x85,
    SIMPLEFOC_VBUSLOW_FAULT =  0x86,
    SIMPLEFOC_CURRENT_FAULT =  0x87,
    SIMPLEFOC_INIT_FAULT =     0xFF
};


class GenericStatus {
public:
    virtual void setStatus(SimpleFOCStatusIndication status);
    virtual SimpleFOCStatusIndication getStatus();
    virtual void init();

protected:
    virtual void setStatusImpl(SimpleFOCStatusIndication status) = 0;
    SimpleFOCStatusIndication status = SimpleFOCStatusIndication::SIMPLEFOC_STATUS_OFF;
};




#endif