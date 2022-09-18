#include <iostream>
#include <string>
#include <stdio.h>
#include "gnss.h"

int main(void){
	



	// GNSS target Site and DOY
	std::string site = "SEOS";
	std::string doy = "032";
	
	GNSS_f Gps;

	// User Position Initiation
	Gps.UserPos << -3042060, 4111978, 3797578, 1;

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

	 


	return 0;

}



