#pragma once
#include <string>
#include <iostream>
#include <vector>
#include <fstream>
#include <cmath>
#include <cstring>

class GNSS_f
{
private:
    std::string Site;
    std::string DOY;

public:

    GNSS_f();

    std::string File_obs;
    
    void setSite(std::string Site);
    void setDOY(std::string DOY);
    void setOBS();
};

