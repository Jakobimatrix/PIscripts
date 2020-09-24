# PIscripts
A collection of scripts which run on my PI. They require specific Hardware to be connected so please read carefully if you want to use them. Also the scripts are not coded respecting "Best Practice" guidelines and do not have the intend to be optimal in any way.

## CPP
To talk to the Hardware I use [WiringPi](http://wiringpi.com/) from Gordon Henderson. Sadly he is a little [pissed](http://wiringpi.com/wiringpi-deprecated/) by some folk and thus stopped working on this awesome project. So if you have any questions about the usage of WiringPi regarding the scripts I wrote, please ask me, dont bother him.

WiringPi is PRE-INSTALLED with standard Raspbian systems.

Every script should have an explaning comment at the begin of the file about the used hardware.

### Build
Each program is a stand alone program and has to be build like this:

``gcc -o the_name_of_the_executable the_name_of_the_script.cpp -lwiringPi``
### Run
Some program might require sudo
``./the_name_of_the_executable``

#### adc_edge_detection.cpp
This program uses an MPC3008 (adc) to detect falling and/or rising edges. It records the time between each detected edge and write the times into a file. It is very accurate. Mind the MPC3008 internal resistance, to have the correct reference GND and Vcc and that you might need bypassing depending on your power supply. Within the script you can define the voltage levels for HIGH and LOW as well as the rising and falling time you expect. Also how long (MINIMAL time) you expect a signal to stay HIGH or LOW. This helps to erase outliners. But you can also use an RC-Lowpass if that suites your situation more. E.g. 0.1uF - 0.01uf and 10Kohm --> fc = 160 - 1600Hz

#### gpio_edge_detection.cpp
This program uses a GPIO and the falling edge detection from wiringPi. It is not as accurate as adc_edge_detection.cpp but much simpler. It writes for each recorded falling edge a timestamp into a file (absolute time). BUT the signal must stay LOW(=GND from PI) for (from some experiments I did) more than 250ms to always capture the edge.

#### shutdownButton.cpp
This program is intended to be started automatically at boot (I recomend /etc/init.d/). Adding a push button to a GPIO (Falling Edge Detection) and an Status LED as well as resistors to reduce the current (see pull down resistor), one can restart or shutdown the PI using the push putton rather than ssh.
