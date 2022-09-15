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

int GNSS_f::str2int(std::string s, int a, int b) {
    // str2double: MATLAB style.
	//int c = b - 3;
	int x = pow(10, std::stoi(s.substr(b - 2, b)));

    return int(std::stod(s.substr(a, 7)) * x); // s.substr(a,b) : read, a~a+7 of string s.
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
		if (line.size() < 67)
		continue;

        std::string line_al = line.substr(60, 9);
        if (line_al.compare("ION ALPHA") == 0){
 			al[0] = str2double(line, 3, 13);
			al[1] = str2double(line, 15, 25);
			al[2] = str2double(line, 27, 37);
			al[3] = str2double(line, 39, 49);
			//std::cout<<"alpha ok"<<std::endl;
            // test.
            //std::cout<<al[0]<<std::endl; // alpha[0]: 1.4900D-08 
        }

        // read beta coef.
        std::string line_be = line.substr(60, 8);
		if (line_be.compare("ION BETA") == 0) {

			be[0] = str2double(line, 3, 13);
			be[1] = str2double(line, 15, 25);
			be[2] = str2double(line, 27, 37);
			be[3] = str2double(line, 39, 49);
			//std::cout<<"beta ok"<<std::endl;
            //std::cout<<be[1]<<std::endl;
            //std::cout<<be[3]<<std::endl; // beta[3]: -6.5540D+04       
		}
        // end of header -> loop break.
		
			std::cout<<line.size()<<std::endl;
			std::string line_tmp = line.substr(60, 13);
			if (line_tmp.compare("END OF HEADER") == 0) {
				//std::cout<<"end: read al be"<<std::endl;
				break;
		}
		
    }

}

double GNSS_f::Find_t_oe(double t){
	return fmod(t, 86400);
}

void GNSS_f::ReadEph(){
    int k= 0;
	
    while (std::getline(input_file, line)){
        double V[31];
        int prn_n = std::stoi(line.substr(0,2));
		
        V[0] = prn_n;
		V[1] = std::stod(line.substr(12, 2)) * 3600 + std::stod(line.substr(15, 2)) * 60 + std::stod(line.substr(18, 2)); // toe
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

		V[9] = str2double(line, 3, 21); // C uc [rad]
		V[10] = str2double(line, 22, 40); // eccentricity
		V[11] = str2double(line, 41, 59); //C us [rad]
		V[12] = str2double(line, 60, 78); //square root A [sqrt(m)]

		// line 4
		std::getline(input_file, line);

		V[13] = Find_t_oe(str2double(line, 3, 21)); // Toe [sec of GPS week]
		V[14] = str2double(line, 22, 40); // C ic [rad]
		V[15] = str2double(line, 41, 59); // Omega 0 [rad]
		V[16] = str2double(line, 60, 78); // C is [rad]

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

		std::string TOO = line.substr(60, 19);
		if (TOO.compare("# / TYPES OF OBSERV") == 0)
		{
			//std::cout << 0;
			double num_sig = str2double2(line, 4, 5);
			// std::cout << num_sig<<std::endl;
			
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
	/*
	for (auto e : num_sigs){
		  std::cout<<e<<std::endl;
	} */

	while (std::getline(input_file, line)) {
		// std::cout<<line<<std::endl;
		if (line.size() < 74){
			continue;
		}

			
		// std::string EOH = line.substr(60, 72); // End Of Header
		
		std::string EOH = line.substr(60, 13); // End Of Header
		// std::cout<<line.size()<<std::endl;
		// std::cout<<EOH<<std::endl;
		if (EOH.compare("END OF HEADER") == 0)
			{
				break;
			}
			
			
	}
	
}

void GNSS_f::ReadObs_Header_Meas(){
	
int first_epoch = 0;
	while (!input_file.eof()) {
		
		// std::cout << " ";
		// std::cout << first_epoch<<std::endl;
			
		/*
		if (first_epoch == 2879)
			std::cout << line;*/

		if (first_epoch == 0) {
			std::getline(input_file, line);
			//first_epoch = 1;
		}
		//else
		//	break;
		if (input_file.peek() == EOF)
			break;


		first_epoch++;

		//if(input_file.eof() == EOF)
			
		//if (getline(input_file, line)) {

		//}
		//else
		//	break;
		//if (first_epoch == 3)
		//	break;

		//std::cout << "new start";
		//std::cout << "\n";
		//std::cout<<line;
		// 해야할 일 str2double 추가 짜기 ( 'E' 없을 때)
		//observation measurement 읽기.


		// std::cout << first_epoch<<std::endl;
		int num_prn = str2int(line, 29, 30);
		//std::cout << num_prn;
		//std::cout << line.size();5

		// Obs ttemp;
		Obs * ttemp = new Obs;
		ttemp -> yy = str2double2(line, 1, 2); 
		ttemp ->mm = str2double2(line, 4, 5);
		ttemp ->dd = str2double2(line, 7, 8);

		ttemp ->hour = str2double2(line, 10, 11);
		ttemp ->min = str2double2(line, 13, 14);
		ttemp ->sec = str2double2(line, 16, 24);

		ttemp ->prn = str2int(line, 30, 31);
		//ttemp.sec = str2double2(line, 4, 5);
		//ttemp.sec = str2double2(line, 4, 5);
		//std::cout << ttemp.prn;
		//std::cout << ttemp.min;

		int cnt_prn = 0;
		//std::cout << cnt_prn;
		// GNSS 읽기,
		//std::string s;
		std::vector<char> gnss_types; // 
		//double *prns = new double[num_prn];
		std::vector<int> prns;
		
		while (cnt_prn < num_prn) {
			
			int line1 = (line.size() - 32) / 3;
			//std::cout << line1;
			for (int iter = 0; iter < line1; iter++) {
				
				//s[cnt_prn] = line[32 + iter * 3];
				gnss_types.push_back(line[32 + iter * 3]);
				//prns[cnt_prn] = str2double2(line, 32 + iter * 3 + 1, 32 + iter * 3 + 2);
				//std::cout<<"Error: here"<<std::endl;
				//std::cout<<line<<std::endl;
				//std::cout<<32 + iter * 3 + 1<<std::endl;
				//std::cout<<32 + iter * 3 + 2<<std::endl;
				//std::cout<<stoi(line.substr(32 + iter * 3 + 1,2))<<std::endl;
				// prns.push_back(str2int(line, 32 + iter * 3 + 1, 32 + iter * 3 + 2));
				prns.push_back(stoi(line.substr(32 + iter * 3 + 1,2)));
				
				//s[cnt_prn] = 
				//Obss.push_back(ttemp);
				cnt_prn++;
				//std::cout << ttemp.prn;
			}
			//break;
			//gnss_type_tmp = lin
			std::getline(input_file, line);
		}
		//std::cout << prns.size();

		//for (int iter = 0; iter < num_prn; iter++) {
			// empty인지 확인
			//아니면 세 개 일고, type / prn number  읽음
			//

		//}
		//delete prns;

		///////////////////////
		//이제 measuremnet를 읽을 차례!
			//?/////////////////
		//std::cout << "sig 개수"; 10
		//std::cout << num_sigs.size();
		//std::cout << line;
		
		double meas_temp;
		int cnt_sig = 0;
		int cnt = 0;
		int jump_line = num_sigs.size() / 5 + 1;
		int iter;
		int push_number = 0;
		for (iter = 0; iter < prns.size() ; iter++) {
			//std::cout << " Line Jump ";
			//std::cout << prns.size() * (num_sigs.size() / 5 + 1) - 1;
			//std::cout << "iter";
			//std::cout << iter;
			for (int j_line = 0; j_line < jump_line; j_line++) {
				//std::cout << "\n";

				if (j_line == (jump_line-1) ) { // 마지막 줄
					// num_sigs[cnt_sig]; // 해당 signal
					
					for (int kk = 0; kk < num_sigs.size() - (jump_line-1)*5; kk++) {
					//	std::cout << "DLFKHJSDLKFJDSL";
						if (line.size() < 16 * kk + 1) continue;
						// if (line.substr(16 * kk + 1, 13) == "            ") continue;
						std::string line_tmp = line.substr(16*kk+1,12);
						if (line_tmp.compare("            ") == 0) continue;
						if (line_tmp.size() == 0) continue;
						
						// std::cout<<line<<std::endl;
						//std::cout<<16*kk+1<<std::endl;
						// std::cout<<line.substr(16*kk+1,13)<<std::endl;
						meas_temp = str2double2(line, 16 * kk + 1,13);
						//std::cout << "\n";
						//std::cout << meas_temp;
						ttemp ->PRN_s.push_back(prns[iter]);
						ttemp ->MEAS_s.push_back(meas_temp);
						ttemp ->PRN_types.push_back(gnss_types[iter]);
						ttemp ->signal_type.push_back(num_sigs[cnt]);
						cnt++;
						push_number++;
					}
				}
				else {
					for (int kk = 0; kk < 5; kk ++ ) {
						
						//std::cout << kk;
						
						
						// std::stod(s.substr(a, b - a + 1))
						if (line.size() < 16 * kk + 1) {
							//std::cout << "E1\n";
							continue;
						}
						

						
						// if(line.substr(16*kk+1,12) == "            ") continue;
						
						std::string line_tmp = line.substr(16*kk+1,12);
						
						if (line_tmp.compare("            ") == 0) continue;
						if (line_tmp.size() == 0) continue;
						// std::cout<<"E3"<<std::endl;
						// std::cout<<line.size()<<std::endl;
						// std::cout<<line_tmp.size()<<std::endl;
						
						//if (std::stod(line.substr(15 * kk + 2, 15 * kk + 13)) == 0) continue;
						//std::cout << "\n";
						//std::cout << kk;
						//std::cout << ":  ";
						//std::cout << line.substr(16 * kk + 1, 13);
						// std::cout<<line.substr(16*kk+1,13)<<std::endl;
						//std::cout << "E2\n";
						
							
						meas_temp = str2double2(line, 16 * kk + 1,13);
						//std::cout << "E3\n";
						//meas_temp = str2double2(line, 16 * kk + 1, 13);
						

						ttemp ->PRN_s.push_back(prns[iter]);
						ttemp ->MEAS_s.push_back(meas_temp);
						ttemp ->PRN_types.push_back(gnss_types[iter]);
						ttemp ->signal_type.push_back(num_sigs[cnt]);
						push_number++;
						cnt++;
					}
					// std::cout << "cnt \n";
				}
				 
				/*
				//^^^^^//  여기서 중단 설정을 걸고, break로 인해 std getline이 먹히는지 안먹히는지 확인을 해보자.
				if (iter == prns.size()) {
					std::cout<<"DFJSLDFJLSDFJSLDFJSDLKFJSLDFJSLDKFJSLDKFJDSLKF";
					//std::getline(input_file, line);
					break;
				}
				else
				
				*/
				if (std::getline(input_file, line)) {}
				else
					break;

				if (input_file.peek() == EOF)
					break;

			} // for jump line
			
			cnt = 0;
		} //for iter
		
		ttemp->pn = push_number;
		Obss.push_back(*ttemp);
		
		delete ttemp;
		// std::cout << "this line test \n ";
	}
	std::cout << "\n End -> Read OBS \n ";

	// test
	/*
	std::cout << "Obss size:"<<sizeof(Obss[0]) - 9 - sizeof(Obss[0].gnss_type)<<std::endl;
	std::cout << "Obss size:"<<Obss[0].pn<<std::endl;
	for (int i = 0; i < Obss[1].pn - 1; i++){
		std::cout<<i;
		std::cout<<"  ";
		std::cout<<Obss[0].PRN_s[i];
		std::cout<<"  ";
		std::cout<<Obss[0].MEAS_s[i];
		std::cout<<"  ";
		std::cout<<Obss[0].PRN_types[i];
		std::cout<<"  ";
		std::cout<<Obss[0].signal_type[i];
		std::cout<<"\n";
	}
	std::cout<<sizeof(Obss[0].PRN_s);
	std::cout<<sizeof(Obss[0].MEAS_s);
	std::cout<<sizeof(Obss[0].PRN_types);
	std::cout<<sizeof(Obss[0].signal_type);
	*/
}


void GNSS_f::ReadObs(std::string fp){
	// input file.
	ReadFile(fp);
	
	ReadObs_Header_Type();

	ReadObs_Header_Meas();
	CloseFile();

}

void GNSS_f::Find_GPS_week_sec(){
	std::cout<<"GPS_week_sec: ";
	GPS_week_sec = now_obs.hour * 3600 + now_obs.min * 60 + now_obs.sec ;
	
	std::cout<<GPS_week_sec;
	std::cout<<"\n";
}
 
double GNSS_f::EccentricityAnomaly(double M_k){
	double E_k;
	E_k = M_k + now_eph.e * sin(M_k);
	for (int i = 1; i < 5; i ++){
		E_k = M_k +  now_eph.e * sin(M_k);
	}

	return E_k;
}

double GNSS_f::Relative_BRDC(){
	double F = 4.442807633e-10;
	double A = pow(now_eph.sqrt_A,2);
	double n_0 = sqrt(gps_mu / pow(A,3));

	double n = n_0 + now_eph.dn;
	double M_k = now_eph.M_0 + n * (GPS_week_sec - now_obs_meas/gps_SoL - now_eph.t_oe);
	double E_k = EccentricityAnomaly(M_k);
	double T_rel = F * now_eph.e * now_eph.sqrt_A * sin(E_k);

	return T_rel;
}

GNSS_f::Sat_Pos_temp GNSS_f::SatPos(){
	// use: now_eph / GPS_week_sec
	double T_k = GPS_week_sec - now_obs_meas / gps_SoL - now_eph.t_oe; // GS - Signal Transmission Time - Toe
	// printf("GS: %6.3f, T_oe: %6.3f \n", GPS_week_sec, now_eph.t_oe);
	double A = pow(now_eph.sqrt_A,2);
	double n_0 = sqrt(gps_mu / pow(A,3));

	double n = n_0 + now_eph.dn;
	double M_k = now_eph.M_0 + n * T_k;
	double E_k = EccentricityAnomaly(M_k);
	double nu_k = atan2((sqrt(1- pow(now_eph.e,2)) * sin(E_k)), cos(E_k)-now_eph.e);
	double Phi_k = nu_k + now_eph.omega;
	
	double d_u_k = now_eph.C_us * sin(2 * Phi_k) + now_eph.C_uc * cos(2 * Phi_k);
	double d_r_k = now_eph.C_rs * sin(2 * Phi_k) + now_eph.C_rc * cos(2 * Phi_k);
	double d_i_k = now_eph.C_is * sin(2 * Phi_k) + now_eph.C_ic * cos(2 * Phi_k);
	
	double u_k = Phi_k + d_u_k;
	double r_k = A * (1 - now_eph.e * cos(E_k)) + d_r_k;
	double i_k = now_eph.i_0 + d_i_k + now_eph.i_dot * T_k;

	double x_k_ = r_k * cos(u_k);
	double y_k_ = r_k * sin(u_k);

	double Omega_k = now_eph.Omega_0 + T_k * (now_eph.Omega_dot - gps_Omega_dot_e) - (gps_Omega_dot_e - now_eph.t_oe);

	double x_k = x_k_ * cos(Omega_k) - y_k_ * cos(i_k) * sin(Omega_k);
	double y_k = x_k_ * sin(Omega_k) + y_k_ * cos(i_k) * cos(Omega_k);
	double z_k = y_k_ * sin(i_k);
	
	// Satellite Rotation.
	double theta = gps_Omega_dot_e * now_obs_meas / gps_SoL;
	double x_k_r = x_k * cos(theta) + y_k * sin(theta);
	double y_k_r = -x_k * sin(theta) + y_k * cos(theta);

	Sat_Pos_temp xyz;

	xyz.x = x_k_r;
	xyz.y = y_k_r;
	xyz.z = z_k;
	
	double T_rel = Relative_BRDC();
	xyz.dt_sat = now_eph.a + now_eph.b * (GPS_week_sec - now_obs_meas / gps_SoL - now_eph.t_oe) + now_eph.c * (GPS_week_sec - now_obs_meas / gps_SoL - now_eph.t_oe) * (GPS_week_sec - now_obs_meas / gps_SoL - now_eph.t_oe) + T_rel- now_eph.t_gd;

	
	
	return xyz;
	// Sat_Pos.push_back(Sat_Pos_temp);
	// printf("PRN: %2d, X: %7.3f, Y: %7.3f, Z: %7.3f \n", now_eph.prn, x_k, y_k, z_k);

}

double L2_Norm_3D(double a, double b, double c){
	return sqrt(pow(a,2) + pow(b,2) + pow(c,2));
}

void GNSS_f::PosEstimation_LS(std::vector<Sat_Pos_temp> Sat_Pos_values){
	for (int k = 0; k<5;k++){
	Eigen::Vector4d H, xHat;
	Eigen::Matrix4d H_tp_H;
	Eigen::Vector4d H_tp_y;

	Eigen::Vector3d LoS_vec, now_UserPos;
	
	 printf("\niter: %2d \n", k);
	now_UserPos[0] = UserPos[0];
	now_UserPos[1] = UserPos[1];
	now_UserPos[2] = UserPos[2];
	
	int PRN_used = 0;

	double LoS;

	double Computed, y;


	for (int i = 0; i<sizeof(Sat_Pos_values);i++){
		LoS_vec << Sat_Pos_values[i].x - now_UserPos[0], Sat_Pos_values[i].y - now_UserPos[1], Sat_Pos_values[i].z - now_UserPos[2];
		
		//LoS = Sat_Pos_values[i].obs - L2_Norm_3D(Sat_Pos_values[i].x,Sat_Pos_values[i].y,Sat_Pos_values[i].z);
		LoS = LoS_vec.norm();
		
		printf("prn: %d, LoS: %6.3f, cdtr: %6.3f, dt_sat: %6.3f \n", Sat_Pos_values[i].prn,LoS, UserPos[3], gps_SoL * Sat_Pos_values[i].dt_sat);
		Computed = LoS + UserPos[3] - gps_SoL * Sat_Pos_values[i].dt_sat;
		y = Sat_Pos_values[i].obs - Computed;
		
		H << -LoS_vec[0]/LoS, -LoS_vec[1]/LoS, -LoS_vec[2]/LoS, 1;
		
		H_tp_H = H_tp_H + H * H.transpose();
		
		H_tp_y = H_tp_y + H * y;
		
		PRN_used++;
	}
	
	xHat = H_tp_H.inverse() * H_tp_y;

	
	now_UserPos[0] = now_UserPos[0] + xHat[0];
	now_UserPos[1] = now_UserPos[1] + xHat[1];
	now_UserPos[2] = now_UserPos[2] + xHat[2];

	if (xHat.norm() < 1e-5){
		UserPos[0] = now_UserPos[0];
		UserPos[1] = now_UserPos[1];
		UserPos[2] = now_UserPos[2];
		
		break;
	}
	}


}

void GNSS_f::gps_L1(){
	std::string tmp;
	std::cout<<"<GPS L1> \n";

	std::vector<Sat_Pos_temp> Sat_Pos_values;

	for (int i = 0; i < now_obs.pn; i++){
		if (now_obs.PRN_types[i] == 'G' && now_obs.signal_type[i] =="C1")
		{
			std::cout<<now_obs.PRN_s[i];
			std::cout<<"  ";
			std::cout<<now_obs.MEAS_s[i];
			std::cout<<"  ";
			std::cout<<now_obs.PRN_types[i];
			std::cout<<"  ";
			std::cout<<now_obs.signal_type[i];
			std::cout<<"\n";
		
				
				
			// ephemeris에서 필요한 정보를 찾기
			for (int j = 0; j< sizeof(ephs);j++){
				// std::cout<<"\n To Do: 내비게이션 파일 첫 데이터가 이전 날짜라서, 시간 +- 3600 내의 데이터만 쓰도록!\n ";

				// time. 현재 시간 이전의 broadcast 값 읽기.
				if (ephs[j].t_oe >= GPS_week_sec && ephs[j].t_oe <= GPS_week_sec + 7200 && ephs[j].prn == now_obs.PRN_s[i])
				{

					now_eph = ephs[j];
					now_obs_meas = now_obs.MEAS_s[i];
					Sat_Pos_temp xyz = SatPos();
					xyz.prn = now_obs.PRN_s[i];
					xyz.obs = now_obs.MEAS_s[i];
					xyz.sig_type = now_obs.signal_type[i];
					Sat_Pos_values.push_back(xyz);
					printf("prn: %2d, x: %6.3f, y: %6.3f, z: %6.3f, obs: %6.3f, dtsat: %6.3f", xyz.prn, xyz.x, xyz.y, xyz.z, xyz.obs, xyz.dt_sat); 
					break;
				}

			}
		}

	}
	// 여기서 값을 확인해보자.
	// 오차 고려하여 Least Squared Estimation으로 사용자 좌표 구하기.
	// 
	PosEstimation_LS(Sat_Pos_values);
	std::cout<<"X: %6.3f, Y: %6.3f, Z: %6.3f \n", UserPos[0], UserPos[1], UserPos[2];

}

void GNSS_f::Positioning(){
	int k = 1;
	for (auto e : Obss)
	{
		k++;

		now_obs = e;
		// 1. Find GPS_week_sec
		Find_GPS_week_sec();

		// 2. Find L1 measurements of the satellite observed at GPS_week_sec
		// 3. and FInd Satellite's position
		gps_L1();

		


		// 4. Find Receiver's position using Least Square Estimation
		//  4.x. consider GNSS errors.
		
		if (k == 2)
		{
			break;
		}
		
	}

	// 5. Output: Statistical Results
}