#pragma once
#include <string>
#include <iostream>
#include <vector>
#include <fstream>
#include <cmath>
#include <cstring>


#include <eigen3/Eigen/Dense>

// Reference: RTKLIB
//           - Solving GPSTime 

class GNSS_f
{
private:
    std::string Site;
    std::string DOY;
	std::string RefSite;
public:

    GNSS_f();

	double gps_Omega_dot_e = 7.2921151467e-5;
	double gps_mu = 3.986005e14; //
	double gps_SoL = 299792458;

	// Time.
	double gpst0[6]={1980,1, 6,0,0,0}; /* gps time reference */
	double epoch2time(int YY, int MM, int DD, int HH, int Min, double Sec);
	// double time2gpst(int YY, int MM, int DD, int HH, int Min, double t, int *week);
	double time2gpst(int YY, int MM, int DD, int HH, int Min, double t);

    // File variation
    std::string File_obs;
    std::string File_nav;

	std::string File_obs_ref;
	std::string File_nav_ref;

    std::ifstream input_file;
    std::string line;

    void setSite(std::string Site);
	void setRefSite(std::string SITE);
    void setDOY(std::string DOY);
    
    double str2double(std::string s, int a, int b);
	double str2double2(std::string s, int a, int b);
	int str2int(std::string s, int a, int b);

    void ReadAlBe(double * al, double * be);

	void ReadFile(std::string fp);
	void CloseFile();
	double DtoE(std::string s, int a, int b);
    void setRINEX(); // rinex file naming.
	void setRINEX_ref();
    void ReadEph();


	void ReadObs_Header_Type();
	void ReadObs_Header_Meas();
	void ReadObs(std::string fp);
	
	double Find_DS(double t);
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
			t_gd = V[27];
			IODC = V[28];
		}
    }; // struct eph.

    std::vector<eph> ephs;

	std::vector<std::string> num_sigs;

	struct Obs {
		int yy, mm, dd, hour, min;

		double sec;
	
		std::string gnss_type; // GRECJ ..
		int prn, pn;
		double meas; // measurement
		
		std::vector<int> PRN_s;
		std::vector<double> MEAS_s;
		std::vector<char> PRN_types;
		std::vector<std::string> signal_type;

		Obs() {
			int yy = 0;
			int mm = 0;
			int dd = 0;

			int hour = 0;
			int min = 0;
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
	struct Sat_Pos_temp{
		double x,y,z;
		int prn;
		double obs;
		double dt_sat;
		std::string sig_type;
		//int num_sat;
	};
	// std::vector<Sat_Pos_temp> Sat_Pos;
	double L2_Norm_3D(double a, double b, double c);
	void Positioning();
	Obs now_obs;
	double now_obs_meas;
	eph now_eph;
	double GPS_week_sec;
	
	double EccentricityAnomaly(double M_k);
	void Find_GPS_week_sec();
	double Relative_BRDC();
	
	void gps_CA();

	Sat_Pos_temp SatPos(); // Calculate Satellite's Position.
	void PosEstimation_LS(std::vector<Sat_Pos_temp> Sat_Pos); // LS: Least Square estimation
	
	Eigen::Vector4d UserPos;


	Obs now_ref_obs;
	void Find_now_ref_obs();

	void gps_L1(); //  RTK

	void ReadUserObs(std::string fp);
	void ReadRefObs(std::string fp);
	void RTK();
	std::vector<Obs> UserObs;
	std::vector<Obs> RefObs;

};

