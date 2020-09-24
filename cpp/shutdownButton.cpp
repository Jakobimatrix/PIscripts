/**
* @file shutdownButton.cpp
* @brief Provides a convenient shut down button in case there is no way to connect via SSH to the PI.
* It has to be called on Boot of the PI.
* It runns in a While(true) loop until system sends SIGTERM.
* It uses GPIO22 on Pin 15 using WiringPI (pin 3) 
* Pressing the Butten will cause an Interupt (on falling Edge): ->let the LED on GPIO27 on Pin 13 using WiringPI (pin 2) blink.
* Pressing the Button and let go within 3 seconds (LED blinks fast) will shut down the system.
* Pressing the Button and let go after 3, before 6 seconds (LED blinks fast, tan after 3 seconds slow) will reboot the system.
* Pressing the button for longer than 6 seconds will let the LED stop blinking and nothing happens.

* @date 11.09.2016
* @author Jakob Wandel
* @version 1.0
**/

extern "C" {
#include <wiringPi.h>//ansprechen der GPIO
}
#include <stdio.h>
#include <cstdlib> //exit()
#include <csignal>
#include <unistd.h>//sleep()
#include <ctime> //time()

using namespace std;

//GLOBALS
int GPIOInterrupt = 3;
int GPIOLED = 2;
bool EXIT = false;
//GLOBALS

void signalHandler( int signum )
{
   EXIT = true;
   exit(0);  
}
void myInterrupt (void){ 
	time_t now = std::time(NULL);
	pinMode(GPIOLED, OUTPUT);
	pinMode(GPIOInterrupt, INPUT);
	digitalWrite(GPIOLED,1);
	sleep(0.5);
	if(digitalRead(GPIOInterrupt) == 0){//der knopf wurde mind 0,5 sec gedrückt--> prevents shutdown from little voltage fluctuations
		while(now + 4 > std::time(NULL)){

			if(now + 2 > std::time(NULL)){	//shutdown
				if(digitalRead(GPIOInterrupt) == 1){  //losgelassen
					digitalWrite(GPIOLED,0);
					system("shutdown &");			
					exit(0);
				}else{
					digitalWrite(GPIOLED, 1);//an
					delay(50);
					digitalWrite(GPIOLED, 0);//aus
					delay(50);
				}
			}else{							//reboot
				if(digitalRead(GPIOInterrupt) == 1){  //losgelassen
					digitalWrite(GPIOLED,0);
					system("reboot &");			
					exit(0);
				}else{
					digitalWrite(GPIOLED, 1);//an
					delay(200);
					digitalWrite(GPIOLED, 0);//aus
					delay(200);
				}
			}
		}
	}
	digitalWrite(GPIOLED,0);
	sleep(1);
	return; 
}
int main ()
{
  // register signal SIGINT and signal handler  
  signal(SIGTERM, signalHandler);  
  signal(SIGINT, signalHandler);

	int wiringPIstatus = wiringPiSetup();//setup GPIO!! must be done before comunication with GPIO
	if(wiringPIstatus == -1){
		return -1;
	}

	wiringPiISR(GPIOInterrupt, INT_EDGE_FALLING, &myInterrupt);
    
	while(!EXIT) 
	{
		sleep(1);	
		waitForInterrupt (GPIOInterrupt,1000) ;
	}
    return 0;
}
