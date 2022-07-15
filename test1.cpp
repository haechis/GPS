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

	Gps.setOBS();	

	return 0;

}
// commit test



