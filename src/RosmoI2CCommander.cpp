
#include "./RosmoI2CCommander.h"
#include "./RosmoESC.h"



bool RosmoI2CCommander::sendRegister(uint8_t motorNum, uint8_t registerNum) {
    switch (registerNum) {
        case REG_LOOP_SPEED:
            writeFloat(loopSpeed);
            break;
        case REG_VBUS:
            writeFloat(vbus.getVoltage());
            break;
        default:
            return I2CCommander::sendRegister(motorNum, registerNum);   // call superclass for the rest
    }
    return true;
};




bool RosmoI2CCommander::receiveRegister(uint8_t motorNum, uint8_t registerNum, int numBytes) {
    switch (registerNum) {
        case REG_LOOP_SPEED: // RO registers
        case REG_VBUS:
            return false;
        default:
            return I2CCommander::receiveRegister(motorNum, registerNum, numBytes);  // other registers handled in superclass
    }
    return true;
};
