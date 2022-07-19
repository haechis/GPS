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

    // File variation
    std::string File_obs;
    std::string File_nav;

    std::ifstream input_file;
    std::string line;

    void setSite(std::string Site);
    void setDOY(std::string DOY);
    
    double str2double(std::string s, int a, int b);

    void ReadAlBe(double * al, double * be);

    void setRINEX(); // rinex file naming.

    // Read RINEX navigation file and save the data
    void ReadEPH(std::string fp);
    
};

