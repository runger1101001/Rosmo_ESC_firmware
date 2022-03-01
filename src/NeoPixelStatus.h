#ifndef __NEO_PIXEL_STATUS_H__
#define __NEO_PIXEL_STATUS_H__


#include "GenericStatus.h"
#include <Adafruit_NeoPixel.h>



class NeoPixelStatus : public GenericStatus {
public:
    NeoPixelStatus(uint8_t pin, uint8_t numPixels, uint16_t pixelType = NEO_GRB + NEO_KHZ800);
    void init() override;
protected:
    void setStatusImpl(SimpleFOCStatusIndication status) override;
    uint8_t pin;
    uint8_t numPixels;
    Adafruit_NeoPixel strip;
};


#endif
