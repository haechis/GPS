#pragma once
#include <string>
#include <iostream>
#include <vector>
#include <fstream>
#include <cmath>
#include <cstring>


#include <eigen3/Eigen/Dense>



class GNSS_f
{
private:
    std::string Site;
    std::string DOY;

public:

    GNSS_f();

	double gps_Omega_dot_e = 7.2921151467e-5;
	double gps_mu = 3.986005e14; // 

    // File variation
    std::string File_obs;
    std::string File_nav;

    std::ifstream input_file;
    std::string line;

    void setSite(std::string Site);
    void setDOY(std::string DOY);
    
    double str2double(std::string s, int a, int b);
	double str2double2(std::string s, int a, int b);
	int str2int(std::string s, int a, int b);

    void ReadAlBe(double * al, double * be);

	void ReadFile(std::string fp);
	void CloseFile();

    void setRINEX(); // rinex file naming.
    void ReadEph();


	void ReadObs_Header_Type();
	void ReadObs_Header_Meas();
	void ReadObs(std::string fp);
	
	double Find_t_oe(double t);
    struct eph {
		char GNSS_type; // G R E C J ...

		int prn;
		double t_oe;
		double a, b, c, t_gd;
		double sqrt_A, e;
		double i_0, i_dot;
		double Omega_dot, omega, Omega_0;
		double dn;
		double M_0;
		double C_uc, C_us;
		double C_rc, C_rs;
		double C_ic, C_is;
		double IODE;
		double week_num;
		double health;
		double IODC;
		eph(){}
		eph(int PPRN, double* V) {
			prn = PPRN;
			t_oe = V[13];
			a = V[2];
			b = V[3];
			c = V[4];
			sqrt_A = V[12];
			e = V[10];
			i_0 = V[17];
			i_dot = V[21];
			Omega_dot = V[20];
			omega = V[19];
			Omega_0 = V[15];
			dn = V[7];
			M_0 = V[8];
			C_uc = V[9];
			C_us = V[11];
			C_rc = V[18];
			C_rs = V[6];
			C_ic = V[14];
			C_is = V[16];
			IODE = V[5];
			week_num = V[23];
			health = V[26];
			IODC = V[28];
		}
    }; // struct eph.

    std::vector<eph> ephs;

	std::vector<std::string> num_sigs;

	struct Obs {
		double yy, mm, dd;

		double hour, min, sec;
	
		std::string gnss_type; // GRECJ ..
		int prn, pn;
		double meas; // measurement
		
		std::vector<int> PRN_s;
		std::vector<double> MEAS_s;
		std::vector<char> PRN_types;
		std::vector<std::string> signal_type;

		Obs() {
			double yy = 0;
			double mm = 0;
			double dd = 0;

			double hour = 0;
			double min = 0;
			double sec = 0;

			std::string gnss_type = ""; // GRECJ ..
			int prn = 0;
			double meas = 0;
			
		}		
	};

	std::vector<Obs> Obss;

	struct MEAS{

	};
	std::vector<MEAS> meas_s;

    // Read RINEX navigation file and save the data
    void ReadNav(std::string fp);
    
	// Satellite's position.
	void Positioning();
	Obs now_obs;
	eph now_eph;
	double GPS_week_sec;
	
	double EccentricityAnomaly(double M_k);
	void Find_GPS_week_sec();
	
	void gps_L1();

	void SatPos(); // Calculate Satellite's Position.
	
};

