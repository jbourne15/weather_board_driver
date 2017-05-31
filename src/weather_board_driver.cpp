#include <wiringPi.h>
#include <wiringPiI2C.h>

#include <iostream>
#include "ros/ros.h"

#include <weather_board_driver/wb_data.h>
#include <weather_board_driver/wb_list.h>

#include <stdio.h>
#include "bme280-i2c.h"
#include "si1132.h"

static unsigned int pressure;
static int temperature;
static unsigned int humidity;
static float altitude;

float SEALEVELPRESSURE_HPA = 1024.25;

#define TCAADDR 0x70
int fd = wiringPiI2CSetup(TCAADDR);
int num_sensors = 3;

void tcaselect(int i){
  if (i>7 || i<0){
    std::cerr<<"not enough multiplexer pins: "<<i<<std::endl;
    return;
  }
  else{    
    wiringPiI2CWrite(fd,1<<i);
  }
}

int main(int argc, char **argv)
{
  char *device = "/dev/i2c-1";
  std::cout<< 'multiplexer handler at: '<< fd <<std::endl;

  for (int j=0; j<num_sensors;j++){
    tcaselect(j);
    si1132_begin(device);
    if (bme280_begin(device) < 0) {
      std::cerr<<"Failed on sensor: "<<j<<std::endl;
      return -1;
    }
  }
  
  ros::init(argc, argv, "weather_board_driver");
  ros::NodeHandle n;
  ros::Publisher wb_pub = n.advertise<weather_board_driver::wb_list>("wb_list", 100);
  ros::Rate loop_rate(50);

  while (ros::ok()) {

    //reset data and list variables
    weather_board_driver::wb_data data;
    weather_board_driver::wb_list list;

    for (int j=0; j<num_sensors;j++){
      tcaselect(j); //select which sensor from multiplexer

      // read in data
      data.UV_index = Si1132_readUV()/100.0;
      data.visible  = Si1132_readVisible();
      data.IR  = Si1132_readIR();
      
      bme280_read_pressure_temperature_humidity(&pressure, &temperature, &humidity);
      data.temperature  = temperature/100.0;
      data.humidity     = humidity/1024.0;
      data.pressure     = pressure/100.0;
      data.altitude     = bme280_readAltitude(pressure,SEALEVELPRESSURE_HPA);

      // push back into list
      list.wb_list.push_back(data);
    }
    
    //std::cout<<list<<std::endl;
    wb_pub.publish(list);

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
