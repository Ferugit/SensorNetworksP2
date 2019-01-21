#include "mbed.h"
#include "SerialGPS.h"

Thread threadGNSS(osPriorityNormal); 


//Declare the global vars needed for communication with main thread
SerialGPS gnss = SerialGPS (PA_9,PA_10,9600);


float lon, lat, hDop, height, geoide, UTCtime;
char *message;//To save the nmea sentency (puntero a la dirección de mem donde está la var)
int delayThreadGNSS;

void gnss_thread();

void gnss_thread() {
	while (1) {
		
		
		if(gnss.sample()){
			lon = gnss.longitude;
			lat = gnss.latitude;
			hDop = gnss.hdop;
			height = gnss.alt;
			geoide = gnss.geoid;
			UTCtime = gnss.time;
			message = gnss.msg;
		}
		
		Thread::wait(delayThreadGNSS*1000);//miliseconds  
	}
}
