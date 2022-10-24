#include <iostream>
#include <string>
#include <stdio.h>
#include "gnss.h"

int main(void){
	
	

	

	// GNSS target Site and DOY
	std::string site = "SEOS";
	std::string doy = "032";
	
	GNSS_f Gps;

	// Unit Test
	//double t = 172800;
	//int weeke = 2195;
	//double k = Gps.time2gpst(2022,2,1,0,0, 0);
	//printf("Unit Test, time to gpst %f \n",k);

	// User Position Initiation
	Gps.UserPos << -3042060, 4111978, 3797578, 1;
	
	// True: -3042060.939 , 4111978.505 , 3797578.579 (ITRF2014)

	Gps.setSite(site);
	Gps.setDOY(doy);

	Gps.setRINEX();

	// create file name to read RINEX navigation file.
    std::string filename(Gps.File_nav);

	// Read navigation file. especially ephemeris 
	Gps.ReadNav(filename);
	
	// create file namte to read RINEX Observation file.
	std::string filename2(Gps.File_obs);
	Gps.ReadObs(filename2);
	

	// Positioning.
	// ...
	std::cout<<Gps.Obss.size()<<std::endl;
	
	// Positioning as much as the number of Obss
	Gps.Positioning();

	// RTK
	std::string RefSite = "DANJ";
	Gps.ReadUserObs(filename2);
	Gps.ReadRefObs(filename2);
	
	Gps.RTK();


	return 0;

}



