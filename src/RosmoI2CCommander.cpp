
#include "./RosmoI2CCommander.h"
#include "./RosmoESC.h"
#include "./hw_setup.h"
#include "communication/SimpleFOCDebug.h"


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


void RosmoI2CCommander::init(uint8_t address){    
    _wire->setClock(400000);          // use same speed on controller device
    _wire->begin(address, true);     // initialize i2c in target mode
    I2CCommander::init(address);
    SimpleFOCDebug::println("RosmoI2CCommander intialized on address ", (int)address);
};


void RosmoI2CCommander::init(){
    uint8_t address = ROSMO_BASE_I2C_ADDRESS;
    pinMode(I2CADDR1_PIN, INPUT_PULLDOWN);
    pinMode(I2CADDR2_PIN, INPUT_PULLDOWN);
    if (digitalRead(I2CADDR1_PIN)==HIGH)
        address += 1;
    if (digitalRead(I2CADDR2_PIN)==HIGH)
        address += 2;
    init(address);
};
