//#include <wiringPi.h>
//#include <wiringPiI2C.h>

#include <iostream>
#include "ros/ros.h"

#include <weather_board_driver/wb_data.h>
#include <weather_board_driver/wb_list.h>

#include <std_msgs/Header.h>

#include <stdio.h>
#include "bme280-i2c.h"
#include "si1132.h"

unsigned int pressure;
int temperature;
unsigned int humidity;
float altitude;

float SEALEVELPRESSURE_HPA = 1024.25;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "single_wb_driver");
  ros::NodeHandle n;

  int dev;
  char *device;
  n.getParam("/dev", dev);
  if (dev==2){
    device = "/dev/i2c-2";
    ROS_INFO("Changed i2c device: %i", 2);
  }
  else{
    device = "/dev/i2c-1";
    ROS_INFO("Changed i2c device: %i", 1);
  }

  std::string topicName = "wb_data";
  n.getParam("/topic_name", topicName);
  

  ros::Publisher wb_pub = n.advertise<weather_board_driver::wb_data>(topicName, 1);
  std::string ns = n.getNamespace();
  const char id   = ns[8];

  int lr;
  n.getParam("/hz", lr);
  std::cout<<"sensor frequency: "<<lr<<" hz"<<std::endl;
  ros::Rate loop_rate(lr);
  //si1132_begin(device);
  if (bme280_begin(device) < 0) {
    std::cerr<<"single bme280 failed"<<std::endl;
    return -1;
  }
  else{
    int com = bme280_set_power_mode(BME280_NORMAL_MODE);
    int com1 = bme280_set_oversamp_pressure(BME280_OVERSAMP_SKIPPED);
    std::cout<<com1<<", "<<com1<<std::endl;
  }

  while (ros::ok()) {

    pressure = 0;
    temperature = 0;
    humidity = 0;
    altitude = 0;

    weather_board_driver::wb_data data;

    // fill in Header
    data.header.stamp = ros::Time::now();
    data.header.seq   = 0;
    data.header.frame_id = ns;
      
    // read in data
    data.UV_index = 0;//Si1132_readUV()/100.0;
    data.visible  = 0;//Si1132_readVisible();
    data.IR  = 0;//Si1132_readIR();
    bme280_read_pressure_temperature_humidity(&pressure, &temperature, &humidity);

    data.temperature  = temperature/100.0;
    data.humidity     = humidity/1024.0;
    data.pressure     = pressure/100.0;
    data.altitude     = bme280_readAltitude(pressure,SEALEVELPRESSURE_HPA);

    // publish data
    wb_pub.publish(data);

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
