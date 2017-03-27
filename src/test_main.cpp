#include "Multi-SensorsDataReceive.h"
#include "iostream"
#include "sys/time.h"
// #include <time.h>
// #include <sstream>
// #include <math.h>
// #include <stdio.h>
// #include <stdlib.h>
// #include <cstring>
// #include <termios.h>
// #include <unistd.h>
// #include <fcntl.h>
/*
#define DEG2RAD(x) ((x)*M_PI/180.0)
#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif*/

// using namespace carto_release;
using namespace std;
struct timeval _start_laser;
struct timeval _end_laser;

 int main(int argc, char **argv)
{  
   //construct a object to test the function of serials
   LaserOdomUart* laser_odom = new LaserOdomUart;
   int timeuse = 0;
   _start_laser.tv_sec = 0;
   _start_laser.tv_usec = 0;
   while(1) {
     //need to add some code to display!
	if(laser_odom->_flag_certain) {
// 	    gettimeofday(&_end_laser, NULL);
// 	    timeuse = 1000000 * ( _end_laser.tv_sec - _start_laser.tv_sec ) + _end_laser.tv_usec -_start_laser.tv_usec;
// 	    cerr<<"Timeuse is    "<<timeuse<<endl;		
	    LASER_ODOM_DATA recv = laser_odom->getCurrentData();
// 	    gettimeofday(&_start_laser, NULL);

	    for(int i=0; i<360; i++) {
		cerr<<"Real Point is   "<<i<<"    X: "<<recv.TmpX[i]<<"      Y: "<<recv.TmpY[i]<<endl;		
	    }
	}
    }
}






