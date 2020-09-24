/*
 +-----+-----+---------+------+---+---Pi 3---+---+------+---------+-----+-----+
 | BCM | wPi |   Name  | MCP  | V | Physical | V | MCP  | Name    | wPi | BCM |
 +-----+-----+---------+------+---+----++----+---+------+---------+-----+-----+
 |  10 |  12 |    MOSI |  11  | 0 | 19 || 20 |   |      | 0v      |     |     |
 |   9 |  13 |    MISO |  12  | 0 | 21 || 22 | 1 |      | GPIO. 6 | 6   | 25  |
 |  11 |  14 |    SCLK |  13  | 0 | 23 || 24 | 1 | 10   | CE0     | 10  | 8   |
 +-----+-----+---------+------+---+---Pi 3---+---+------+---------+-----+-----+

 MCPC 3008

      +-\/-+
 ch0 1|    |16  Vcc (2.7V - 5.5V)
 ch1 2|    |15  Vref
 ch2 3|    |14  GNDref
 ch3 4|    |13  SCLK
 ch4 5|    |12  MISO
 ch5 6|    |11  MOSI
 ch6 7|    |10  CS
 ch7 8|    |9   GND (0V)
      +----+

200 ksps max. sampling rate at VDD=5V
75 ksps max. sampling rate at VDD=2.7V

// wiring pi will probably introduce some delay (in comparison to using plane c and talkking spi directly)
*/

extern "C" {
#include <wiringPi.h>
#include <mcp3004.h> // supports 3008 too!
}
#include <stdio.h>
#include <stdlib.h>
#include <csignal>
#include <iostream>
#include <fstream>
#include <chrono>
#include <string> 
#include <thread>
#include <iostream>
#include <algorithm>

// HIGH and LOW are already defined in wiringPi and C can not have the same name in different enums / defines
enum State{
  HIGH_, LOW_, UNKNOWN_
};

struct Edge{
  long min_rise_time_ms;
  long min_fall_time_ms;
  float trigger_v_low;
  float trigger_v_high;

  const State getState(float v){
    if(v > trigger_v_high){
      return State::HIGH_;
    }
    if(v < trigger_v_low){
      return State::LOW_;}
    return State::UNKNOWN_;
  }
};


//GLOBALS
constexpr int SPI_CHAN = 0;
constexpr int ADC_PIN_BASE = 12345; // this needs to be higher than the highst avaiable pin
constexpr float V_REF = 5.0;
constexpr float RESOLUTION_BIT = 1024;
bool EXIT = false;
constexpr long READ_DELAY_MS = 1;
constexpr int ADC_CHANNEL = 0;
std::ofstream outfile;
std::string FILE_PATH = "interrupt.csv";
//GLOBALS

void writeToFile(const std::string& s){
	outfile << s;
	outfile << "\n";
}


int readChannelValue(int channel){
  return analogRead(ADC_PIN_BASE + channel);
}

float readChannel(int channel){
  int read_value = readChannelValue(channel);
  float f = V_REF/RESOLUTION_BIT;
  return f*static_cast<float>(read_value);
}

long getCurrentMs(){
	auto now = std::chrono::high_resolution_clock::now();
	auto now_ms = std::chrono::time_point_cast<std::chrono::milliseconds>(now);
	auto epoch = now_ms.time_since_epoch();
	auto value = std::chrono::duration_cast<std::chrono::milliseconds>(epoch);
 	return value.count();
}

void signalHandler( int signum )
{
   EXIT = true;
}

void sleepFor(long sleep_ms){
  std::chrono::milliseconds milli(sleep_ms);
  std::this_thread::sleep_for(milli);
}

void sleepFor(long milli_sleep, long start_ms){
  long now_ms = getCurrentMs();
  long diff = now_ms - start_ms;
  if(diff > 0){
    sleepFor(diff);
  }
}

/*
 * clean case:
 *  (1)|  _____ _
 * (-1)|       _ ______
 *     |      (a) (b)
 * (a) actual change
 * (b) trigger whatever should happen at change
 * (b) accept change if at state for at least min_rise/fall_time_ms
 *
 * unclean case:
 *  (1)| ___ _
 *  (0)|    ^ ^ v (c)
 * (-1)|       _ _____
 *            (a) (b)
 * (c) the state (0) will always be interpreded as the last state (1/-1) which was measured.
 * */
void runEdgeDetection(Edge &edge, int channel){
  
  int min_buffer_length_detect_rise = std::max(edge.min_rise_time_ms/READ_DELAY_MS,1l);
  int min_buffer_length_detect_fall = std::max(edge.min_fall_time_ms/READ_DELAY_MS,1l);
  int buffer_length = std::max(min_buffer_length_detect_fall, min_buffer_length_detect_rise) + 1;
 
  if(buffer_length == 2){
    std::cout << "WARNING: READ_DELAY (" << READ_DELAY_MS << ") is too long to detect Edges robustly. Outliners might end up as a detection. The specified rise / fall time sould be at least three times longer than the READ_DELAY_MS"<<std::endl; 
  }else if(buffer_length < 1){
    std::cout << "ERROR specified READ_DELAY_MS or rise / fall time in given Edge &edge is zero or negative"<<std::endl;
  }

  float* measurement_buffer;
  measurement_buffer = new float[buffer_length];
  State* state_buffer;
  state_buffer = new State[buffer_length];
  long* timestamp_buffer;
  timestamp_buffer = new long[buffer_length];

  std::fill(state_buffer, state_buffer + buffer_length, State::UNKNOWN_);

  auto circularIpp = [&](int &id){
    id++;
    if(id >= buffer_length){
      id = 0;
    }
  };
  auto circularImm = [&](int &id){
    id--;
    if(id < 0){
      id = buffer_length - 1;
    }
  };

  auto readChannelFillArrays = [&](int current_id){
    timestamp_buffer[current_id] = getCurrentMs();
    measurement_buffer[current_id] = readChannel(channel);                         
    state_buffer[current_id] = edge.getState(measurement_buffer[current_id]);
  };

  auto readChannelFillArraysHighLow = [&](int current_id, State last_measured_state){
    readChannelFillArrays(current_id);
    if(state_buffer[current_id] == State::UNKNOWN_){
      state_buffer[current_id] = last_measured_state;
    }
  };

  auto getCurrentState = [&](int current_id){
    int count_consecutive_high = 0;
    int count_consecutive_low = 0;
    int count_unknown = 0;
    for(int i = 0; i < buffer_length; i++){
      if(state_buffer[current_id] == State::HIGH_){
        count_consecutive_low = 0;
        count_consecutive_high++;
        count_consecutive_high += count_unknown;
        count_unknown = 0;
        if(count_consecutive_high >= min_buffer_length_detect_rise){
          return State::HIGH_;
        }
      }else if(state_buffer[current_id] == State::LOW_){
        count_consecutive_high = 0;
        count_consecutive_low++;
        count_consecutive_low += count_unknown;
        count_unknown = 0;
        if(count_consecutive_low >= min_buffer_length_detect_fall){
          return State::LOW_;
        }
      }else{
        count_unknown++;
      }
      circularImm(current_id);
    }
    return State::UNKNOWN_;
  };
  
  
  std::cout <<"Edge Detection initialization: Take care that the state of the measured channel is either high or low."<<std::endl;
  // fill the very first state, it musten't be UNKNOWN_
  do{
    readChannelFillArrays(0);  
  }while(state_buffer[0]  == State::UNKNOWN_);
  //Fill the rest
  State start_state = State::UNKNOWN_;
  State last_measured_state = state_buffer[0];
  int current_id = 1;
  while(start_state == State::UNKNOWN_){
    sleepFor(READ_DELAY_MS);
    readChannelFillArraysHighLow(current_id, last_measured_state);
     start_state = getCurrentState(current_id);
     last_measured_state = state_buffer[current_id];
     circularIpp(current_id);
  }

  auto edgeDetection = [&](int current_id, State is_state, long &edge_ms){
    if(last_measured_state == State::UNKNOWN_){
      std::cout << "Warning::edgeDetection: can not detect edge if current State is unknown." << std::endl;
      return false;
    }
    int min_num_detect = (is_state == State::HIGH_) ? min_buffer_length_detect_fall : min_buffer_length_detect_rise;
    State changed_state = (is_state == State::HIGH_) ? State::LOW_ : State::HIGH_; 
    for(int i = 0; i < min_num_detect; i++){
      if(changed_state != state_buffer[current_id]){
        return false;
      }
      circularImm(current_id);
    }
    if(is_state == state_buffer[current_id]){
      // get the best estimation of the edge
      if(edge.getState(measurement_buffer[current_id]) != State::UNKNOWN_){
        circularIpp(current_id);
      }
      edge_ms = timestamp_buffer[current_id];
      return true;
    }
    return false;
  };


  State is_state = start_state;
  long start;
  std::cout << "Initialization finnished" <<std::endl;
  long last_edge_ms = getCurrentMs();
  while(!EXIT){
    start = getCurrentMs();
    readChannelFillArraysHighLow(current_id, last_measured_state);
    last_measured_state = state_buffer[current_id];
    long edge_ms;
    if(edgeDetection(current_id, is_state, edge_ms)){
  
      if(is_state == State::HIGH_){
        long time_ms = edge_ms - last_edge_ms;
        last_edge_ms = edge_ms;
        std::string s = std::to_string(time_ms);      
        writeToFile(s);
        is_state = State::LOW_;
      }else{
        //writeToFile(s);
        is_state = State::HIGH_;
      }
    
    }
    circularIpp(current_id);
    sleepFor(READ_DELAY_MS, start);
  }
  
  delete[] state_buffer;
  delete[] timestamp_buffer;
  delete[] measurement_buffer;
}

int main(void)
{
  signal(SIGTERM, signalHandler);
  signal(SIGINT, signalHandler);
	
  outfile.open(FILE_PATH, std::ios_base::app);
	if(!outfile.is_open()){
		std::cout << "Failed to open " << FILE_PATH <<std::endl;
		return 1;
	}

  int wiringPIstatus = wiringPiSetup();
  if(wiringPIstatus == -1){
     std::cout<<"WiringPi setup failed"<<std::endl;
     return -1;
  }
  Edge edge = Edge();
  edge.min_fall_time_ms = 5;
  edge.min_rise_time_ms = 5;
  edge.trigger_v_high = 3.5;
  edge.trigger_v_low = 2.5;	

  mcp3004Setup (ADC_PIN_BASE, SPI_CHAN);
  runEdgeDetection(edge, ADC_CHANNEL);
  outfile.flush();
  outfile.close();
  std::cout << "END"<<std::endl;
  return 0;
}
