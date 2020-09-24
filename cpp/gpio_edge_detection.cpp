/**
* @file measurePulses.cpp
* @brief Writes each pull to ground on GPIO26 (37) WiringPi (pin 25) in to a file with timestamp.
* It runns in a While(true) loop until terminated

* @date 28.08.2020
* @author Jakob Wandel
* @version 1.0
**/

extern "C" {
#include <wiringPi.h>//ansprechen der GPIO
}
#include <stdio.h>
#include <cstdlib> //exit()
#include <csignal>
#include <fstream>
#include <chrono>
#include <string> 
#include <thread>
#include <iostream>

//GLOBALS
int GPIOInterrupt = 25;
bool EXIT = false;
std::ofstream outfile;
std::string FILE_PATH = "interrupt.csv";
long START;
//GLOBALS

void signalHandler( int signum )
{
   EXIT = true;
}

void writeToFile(const std::string& s){
	outfile << s;
	outfile << "\n";
        std::cout << s <<std::endl;
}

long getCurrentMs(){
	auto now = std::chrono::high_resolution_clock::now();
	auto now_ms = std::chrono::time_point_cast<std::chrono::milliseconds>(now);
	auto epoch = now_ms.time_since_epoch();
	auto value = std::chrono::duration_cast<std::chrono::milliseconds>(epoch);
 	return value.count();
}

void myInterrupt (void){ 
	long duration = getCurrentMs();
	std::string s = std::to_string(duration);
	writeToFile(s);
}

int main ()
{	
	START = getCurrentMs();
	outfile.open(FILE_PATH, std::ios_base::app);

	if(!outfile.is_open()){
		std::cout << "Failed to open " << FILE_PATH <<std::endl;
		return 1;
	}

    	// register signal SIGINT and signal handler  
    	signal(SIGTERM, signalHandler);  

	int wiringPIstatus = wiringPiSetup();//setuo GPIO!! must be done before comunication with GPIO
	if(wiringPIstatus == -1){
		return -1;
	}
  	if (wiringPiISR (GPIOInterrupt, INT_EDGE_FALLING, &myInterrupt) < 0)
  	{
		std::cout << "unable to install interrupt"<<std::endl;
    		return 1 ;
  	}
    	writeToFile("---");
	std::chrono::milliseconds milli(20);
	while(!EXIT) 
	{
		std::this_thread::sleep_for(milli);
	}
	outfile.flush();
	outfile.close();
    return 0;
}
