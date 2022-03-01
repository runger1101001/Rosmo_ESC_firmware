

#include "./NeoPixelStatus.h"


NeoPixelStatus::NeoPixelStatus(uint8_t pin, uint8_t numPixels, uint16_t pixelType) : pin(pin), numPixels(numPixels), strip(numPixels, pin, pixelType) {
};


void NeoPixelStatus::setStatusImpl(SimpleFOCStatusIndication status){
    switch (status) {
    SIMPLEFOC_STATUS_OFF:
        strip.setPixelColor(0, 0, 0, 0);
        break;
    SIMPLEFOC_STATUS_ON:
        strip.setPixelColor(0, 0, 255, 0);
        break;
    SIMPLEFOC_INITIALIZING:
    SIMPLEFOC_CALIBRATING:
        strip.setPixelColor(0, 195, 9, 232);
        break;
    SIMPLEFOC_FAULT:
    SIMPLEFOC_MOTOR_FAULT:
    SIMPLEFOC_SENSOR_FAULT:
    SIMPLEFOC_COMM_FAULT:
    SIMPLEFOC_TEMP_FAULT:
    SIMPLEFOC_CURRENT_FAULT:
        strip.setPixelColor(0, 255, 0, 0);
        break;
    SIMPLEFOC_INIT_FAULT:
        strip.setPixelColor(0, 232, 94, 9);
        break;
    SIMPLEFOC_VBUSHIGH_FAULT:
    SIMPLEFOC_VBUSLOW_FAULT:
        strip.setPixelColor(0, 232, 221, 9);
        break;
    }
    strip.show();
};


void NeoPixelStatus::init(){
    strip.begin();
    pinMode(pin, OUTPUT_OPEN_DRAIN);
    strip.setBrightness(255);
    strip.setPixelColor(0, 0, 0, 0);
    strip.show();
};


