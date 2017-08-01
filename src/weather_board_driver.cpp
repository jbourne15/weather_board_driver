#include <wiringPi.h>
#include <wiringPiI2C.h>

#include <iostream>
#include "ros/ros.h"

#include <std_msgs/Header.h>

#include <weather_board_driver/wb_data.h>
#include <weather_board_driver/wb_list.h>

#include <stdio.h>
#include <vector>
#include <numeric>
#include "bme280-i2c.h"
#include "si1132.h"

unsigned int pressure;
int temperature;
unsigned int humidity;
float altitude;

float SEALEVELPRESSURE_HPA = 1024.25;

#define TCAADDR1 0x70 //first multiplexer
#define TCAADDR2 0x71 //second multiplexer
#define TCAADDR3 0x72 //third multiplexer
#define TCAADDR4 0x73 //third multiplexer

// file handlers for the multiplexers
int fd1; 
int fd2;
int fd3;
int fd4;

void tcaselect(int i){
  int sensor_i = i%8; // find sensor number
  //std::cout<<"multiplexer #: "<<i/8<<std::endl;
  switch (i/8){
  case 0:
    wiringPiI2CWrite(fd2,0); //no channel selected from 2nd multiplexer
    wiringPiI2CWrite(fd3,0); //no channel selected from 3nd multiplexer
    wiringPiI2CWrite(fd4,0); //no channel selected from 4nd multiplexer
    wiringPiI2CWrite(fd1,1<<sensor_i); //select sensor from 1st multiplexer    
    break;
  case 1:
    wiringPiI2CWrite(fd1,0); //no channel selected from 1nd multiplexer
    wiringPiI2CWrite(fd3,0); //no channel selected from 3nd multiplexer
    wiringPiI2CWrite(fd4,0); //no channel selected from 4nd multiplexer
    wiringPiI2CWrite(fd2,1<<sensor_i); //select sensor from 1st multiplexer    
    break;
  case 2:
    wiringPiI2CWrite(fd1,0); //no channel selected from 1nd multiplexer
    wiringPiI2CWrite(fd2,0); //no channel selected from 2nd multiplexer
    wiringPiI2CWrite(fd4,0); //no channel selected from 4nd multiplexer
    wiringPiI2CWrite(fd3,1<<sensor_i); //select sensor from 1st multiplexer    
    break;
  case 3:
    wiringPiI2CWrite(fd1,0); //no channel selected from 1nd multiplexer
    wiringPiI2CWrite(fd2,0); //no channel selected from 2nd multiplexer
    wiringPiI2CWrite(fd3,0); //no channel selected from 3nd multiplexer
    wiringPiI2CWrite(fd4,1<<sensor_i); //select sensor from 1st multiplexer    
    break;

  }
    
}


int main(int argc, char **argv)
{

  ros::init(argc, argv, "weather_board_driver");
  ros::NodeHandle n;
  ros::Publisher wb_pub = n.advertise<weather_board_driver::wb_list>("wb_list", 1);
  
  std::vector<float> biasHlist;
  bool  biasFlag  = false; // flag designates when to publish wb data
  float biasMean  = 0;     // container for holding the humidity mean across sensors for many counts (biasNum)
  int   biasCount = 0;     // counter   
  int   biasNum   = 0;     // quantifies how many iterates to take average over
  
  n.getParam("/biasNum", biasNum);
  int num_sensors = 0;
  n.getParam("/num_sensors", num_sensors);
  std::vector<float> bias(num_sensors,0);
  
  char *device;// stores the device path according to the devID
  int devID; // reads in ROS parameter integeter designating which device
  n.getParam("/device", devID);
  if (devID==1){
    device = "/dev/i2c-1";
  }
  else if (devID==2){
    device = "/dev/i2c-2";
  }
  
  int lr;
  n.getParam("/sensorhz", lr);
  ros::Rate loop_rate(lr);

  fd1 = wiringPiI2CSetupInterface(device, TCAADDR1);
  fd2 = wiringPiI2CSetupInterface(device, TCAADDR2);
  fd3 = wiringPiI2CSetupInterface(device, TCAADDR3);
  fd4 = wiringPiI2CSetupInterface(device, TCAADDR4);
  std::cout<<"starting 4 multiplexers at handle: "<<fd1<<", "<<fd2<<", "<<fd3<<", "<<fd4<<std::endl;
  
  std::cout<<"starting sensors ";
  for (int j=0; j<num_sensors;j++){
    tcaselect(j);
    std::cout<<j<<", ";
    if (j==num_sensors-1) {std::cout<<j<<std::endl;}
    
    //si1132_begin(device);
    if (bme280_begin(device) < 0) {
      std::cerr<<"bme280 failed on sensor: "<<j<<std::endl;
      return -1;
    }
    else{
      int com = bme280_set_power_mode(BME280_NORMAL_MODE);
      int com1 = bme280_set_oversamp_pressure(BME280_OVERSAMP_SKIPPED);
      //std::cout<<com1<<", "<<com1<<std::endl;
    }
   }
  
  while (ros::ok()) {

    //reset list variables
    weather_board_driver::wb_list list;
    
    for (int j=0; j<num_sensors;j++){
      pressure = 0;
      temperature = 0;
      humidity = 0;
      altitude = 0;
      tcaselect(j);

      weather_board_driver::wb_data data;

      // fill in Header
      data.header.stamp = ros::Time::now();
      data.header.seq   = j;
      
      // read in data
      data.UV_index = 0;//Si1132_readUV()/100.0;
      data.visible  = 0;//Si1132_readVisible();
      data.IR  = 0;//Si1132_readIR();
      bme280_read_pressure_temperature_humidity(&pressure, &temperature, &humidity);

      data.temperature  = temperature/100.0;
      data.humidity     = humidity/1024.0 + bias[j]; // bias is initially 0 until biasFlag is True
      data.pressure     = pressure/100.0;
      data.altitude     = bme280_readAltitude(pressure,SEALEVELPRESSURE_HPA);

      // push back into list
      list.wb_list.push_back(data);

      // initially gather humidity data to correct for the bias.
      if (biasFlag==false){
	biasHlist.push_back(data.humidity);
      }
    }
    double mean=0;
    if (biasFlag!=true){
      double sum = std::accumulate(biasHlist.begin(), biasHlist.end(), 0.0);
      mean = sum / biasHlist.size();
    }    

    if (biasFlag == true){ //publish data+(bias!=0)
      wb_pub.publish(list);
    } else { // update running mean     
      biasMean += mean;
      if (biasCount+1>=biasNum and biasFlag!=true){//check if we have enough data 
	biasFlag = true;
	if (biasNum!=0){std::cout<<"bias has been corrected"<<std::endl;}
	biasMean = biasMean*1.0/biasNum;
	if (biasNum!=0){ // require biasNum>0 else bias is 0 forall sensors (initialized)
	  for (int j=0; j<num_sensors;j++){	  
	    bias[j] = biasMean - biasHlist[j];
	  }
	}
      }
    }

    ros::spinOnce();
    loop_rate.sleep();

    if (biasFlag != true){biasCount+=1;}
  }
  return 0;
}
