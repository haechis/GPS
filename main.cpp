#include <iostream>
#include <string>
#include <stdio.h>
#include "gnss.h"

int main(void){
	
	// GNSS target Site and DOY
	std::string site = "SEOS";
	std::string doy = "032";
	
	GNSS_f Gps;

	Gps.setSite(site);
	Gps.setDOY(doy);

	Gps.setRinex();	

	// create file name to read RINEX files.
    std::string filename(Gps.File_nav);

	// Read navigation file. especially ephemeris 
	Gps.ReadEPH(filename);

	return 0;

}
// commit test


