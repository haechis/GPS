#include "gnss.h"

GNSS_f::GNSS_f() {};

void GNSS_f::setSite(std::string Site){
    // Set Site name
    this-> Site = Site;
}

void GNSS_f::setDOY(std::string DOY){
    // Set DOY
    this -> DOY = DOY;
}

void GNSS_f::setOBS(){
    // create file name.
    File_obs = this-> Site + this->DOY + "0.22o";
    
    // - test -
    printf("test ok\n");
}