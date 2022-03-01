

#include "./GenericStatus.h"



void GenericStatus::setStatus(SimpleFOCStatusIndication status){
    this->status = status;
    setStatusImpl(status);
};



SimpleFOCStatusIndication GenericStatus::getStatus(){
    return status;
};



void GenericStatus::init(){
    // nothing to do here  
};