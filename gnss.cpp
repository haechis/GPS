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

void GNSS_f::setRINEX(){
    // create file name.
    File_obs = this-> Site + this-> DOY + "0.22o";
    File_nav = this -> Site + this -> DOY + "0.22n";
    // - test -
    printf("<test> File_obs: %s\n",File_obs.c_str()); // c++에서 printf로 string 출력시, .c_str 사용해야 함.
    printf("<test> File_nav: %s\n",File_nav.c_str()); 
}


double GNSS_f::str2double(std::string s, int a, int b) {
    // str2double: MATLAB style.
	//int c = b - 3;
	double x = pow(10, std::stof(s.substr(b - 2, b)));

    //    printf("앞 자리: %6.3f, 뒷 자리: %10.7f, 곱: %10.7f \n",std::stod(s.substr(a, b - 4)),x, std::stod(s.substr(a, b - 4))*x);
    std::cout<<std::stod(s.substr(a, b - 4))<<std::endl;
	return std::stod(s.substr(a, b - 4)) * x;

}


void GNSS_f::ReadEPH(std::string fp){
    std::string line;

    // input file.
    std::ifstream input_file(fp);
    if (!input_file.is_open()){
        std::cerr << "Could not open the file - ' " << fp << std::endl;
    }
    else{
        printf("Success to read RINEX navigation file\n");
    }

    // Klobuchar Coefficient Array.
    // Klobuchar Coef. 
    double al[] = {0.0, 0.0, 0.0, 0.0};
    double be[] = {0.0, 0.0, 0.0, 0.0};


    // Read Klobuchar Coef. and End of header
    while (std::getline(input_file, line)){
        // read alpha coef.
        std::string line_al = line.substr(60,68);
        if (line_al.compare("ION ALPHA") == 0){
 			al[0] = str2double(line, 3, 13);
			al[1] = str2double(line, 15, 25);
			al[2] = str2double(line, 27, 37);
			al[3] = str2double(line, 39, 49);

            // test.
            
            
            std::cout<<al[1]<<std::endl;
            std::cout<<al[2]<<std::endl;
            std::cout<<al[3]<<std::endl;
            std::cout<<al[0]<<std::endl; // alpha[0]: 1.4900D-08 
        }

        // read beta coef.
        std::string line_be = line.substr(60, 67);
		if (line_be.compare("ION BETA") == 0) {

			be[0] = str2double(line, 3, 13);
			be[1] = str2double(line, 15, 25);
			be[2] = str2double(line, 27, 37);
			be[3] = str2double(line, 39, 49);

            //std::cout<<be[1]<<std::endl;
            //std::cout<<be[3]<<std::endl; // beta[3]: -6.5540D+04       
		}
        // end of header -> loop break.
		std::string line_tmp = line.substr(60, 72);
		if (line_tmp.compare("END OF HEADER") == 0) {
			break;
		}



           

    }
}