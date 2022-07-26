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

    return std::stod(s.substr(a, 7)) * x; // s.substr(a,b) : read, a~a+7 of string s.
}

double GNSS_f::str2double2(std::string s, int a, int b) {

	//double x =
		return std::stof(s.substr(a, b - a + 1));
}


void GNSS_f::ReadFile(std::string fp){
    // input file.
    input_file.open(fp);
    if (!input_file.is_open()){
        std::cerr << "Could not open the file - ' " << fp << std::endl;
    }
    else{
        printf("Success to read RINEX navigation file\n");
    }

}

void GNSS_f::CloseFile(){
	input_file.close();
}

void GNSS_f::ReadAlBe(double * al, double * be){
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
            //std::cout<<al[0]<<std::endl; // alpha[0]: 1.4900D-08 
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

void GNSS_f::ReadEph(){
    int k= 0;
    while (std::getline(input_file, line)){
        double V[31];
        int prn_n = std::stoi(line.substr(0,2));
        V[0] = prn_n;
		V[1] = std::stod(line.substr(12, 13)) * 3600 + std::stod(line.substr(15, 16)) * 60 + std::stod(line.substr(18, 21)); // toe
		V[2] = str2double(line, 22, 40); // a [sec]
		V[3] = str2double(line, 41, 59); // b [sec/sec]
		V[4] = str2double(line, 60, 78); // c [sec/sec2]
		//std::cout << V[3] << std::endl;

		// line 2
		std::getline(input_file, line);

		V[5] = str2double(line, 3, 21); // IODE
		V[6] = str2double(line, 22, 40); // Crs [meters]
		V[7] = str2double(line, 41, 59); // dn [rad/sec]
		V[8] = str2double(line, 60, 78); // M0 [rad]
		//std::cout << V[8] << std::endl;

		// line 3
		std::getline(input_file, line);

		V[9] = str2double(line, 3, 21); // Cuc [rad]
		V[10] = str2double(line, 22, 40); // eccentricity
		V[11] = str2double(line, 41, 59); //Cus [rad]
		V[12] = str2double(line, 60, 78); //sqrt(A) [sqrt(m)]

		// line 4
		std::getline(input_file, line);

		V[13] = str2double(line, 3, 21); // Toe [sec of GPS week]
		V[14] = str2double(line, 22, 40); // Cic [rad]
		V[15] = str2double(line, 41, 59); // Omega 0 [rad]
		V[16] = str2double(line, 60, 78); // Cis [rad]

		// line 5
		std::getline(input_file, line);

		V[17] = str2double(line, 3, 21); //i0 [rad]
		V[18] = str2double(line, 22, 40); // Crc [meters]
		V[19] = str2double(line, 41, 59); // omega [rad]
		V[20] = str2double(line, 60, 78); // Omega dot [rad/sec]

		// line 6
		std::getline(input_file, line); 

		V[21] = str2double(line, 3, 21); // IDOT [rad/sec]
		V[22] = str2double(line, 22, 40); // Codes on L2 channel
		V[23] = str2double(line, 41, 59); // GPS Week #
		V[24] = str2double(line, 60, 78); // L2P data flag

		// line 7
		std::getline(input_file, line);

		V[25] = str2double(line, 3, 21); // SV accuracy
		V[26] = str2double(line, 22, 40); // SV health
		V[27] = str2double(line, 41, 59); // TGD [sec]
		V[28] = str2double(line, 60, 78); // IODC

		// line 8
		std::getline(input_file, line);

		V[29] = str2double(line, 3, 21); // transmission time of message
		V[30] = str2double(line, 22, 40);
		//V[31] = str2double(line, 41, 59);
		//V[32] = str2double(line, 60, 78);

		ephs.push_back(eph(prn_n, V));

    } // end while.
}

void GNSS_f::ReadNav(std::string fp){

	ReadFile(fp);

    // Klobuchar Coefficient Array.
    // Klobuchar Coef. 
    double al[] = {0.0, 0.0, 0.0, 0.0};
    double be[] = {0.0, 0.0, 0.0, 0.0};

    ReadAlBe(al, be);
    //std::cout<<be[3]<<std::endl;

    // Read Navigation data (Ephemeris)
    ReadEph();
    // test of ReadEPH.
    // std::cout<<ephs[1].C_ic;
    /*for (auto e : ephs){
        std::cout<<e.prn<<std::endl;
    }*/
	CloseFile();
}

void GNSS_f::ReadObs_Header_Type(){

	while (std::getline(input_file, line)) {

		std::string TOO = line.substr(60, 78);
		if (TOO.compare("# / TYPES OF OBSERV") == 0)
		{
			//std::cout << 0;
			double num_sig = str2double2(line, 4, 5);
			std::cout << num_sig<<std::endl;
			
			int cnt_sig = 0;
			while (cnt_sig < num_sig) {
				for (int iter = 0; iter < 9; iter++) {
					//std::cout << line.substr(10 + iter * 6, 2);
					//std::cout << "          ";
					// num_sigs.push_back(line.substr(10 + iter * 6, 10 * iter * 6 + 2));
					num_sigs.push_back(line.substr(10 + iter * 6, 2)); // revision: b of substr
					cnt_sig++;

					if (cnt_sig == int(num_sig))
					{
						std::cout << "here~\n";
						break;
					} // 돌다가 같아지면 break.
				}

				if (cnt_sig < int(num_sig - 1))
				{

					std::getline(input_file, line);
				}
				if (cnt_sig == num_sig - 1)
					break;

			}

			break;
			// break;
			if (cnt_sig == num_sig - 1)
				break;
		}
	}


	// test
	for (auto e : num_sigs){
		  std::cout<<e<<std::endl;
	}

	while (std::getline(input_file, line)) {
		// std::string EOH = line.substr(60, 72); // End Of Header
		std::string EOH = line.substr(60, 12); // End Of Header
		if (EOH.compare("END OF HEADER") == 0)
			{
				std::cout<<"test";
				break;
			}
			
	}
}

void GNSS_f::ReadObs_Header_Meas(){
	




}


void GNSS_f::ReadObs(std::string fp){
	// input file.
	ReadFile(fp);
	
	ReadObs_Header_Type();

	ReadObs_Header_Meas();
	CloseFile();

}