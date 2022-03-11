#ifndef __ROSMOI2CCOMMANDER_H__
#define __ROSMOI2CCOMMANDER_H__







#include "comms/i2c/I2CCommander.h"



typedef enum : uint8_t {
    REG_LOOP_SPEED = 0x80,  // float, read-only
    REG_VBUS = 0x81,        // float, read-only
} RosmoRegister;





/**
 * Extends the standard I2CCommander with board-specific functions.
 */
class RosmoI2CCommander : public I2CCommander {
    public:
        RosmoI2CCommander(TwoWire* wire = &Wire) : I2CCommander(wire) {};
        ~RosmoI2CCommander(){};

        float loopSpeed = 0.0f;

    protected:
        virtual bool sendRegister(uint8_t motorNum, uint8_t registerNum) override;
        virtual bool receiveRegister(uint8_t motorNum, uint8_t registerNum, int numBytes) override;
};






#endif