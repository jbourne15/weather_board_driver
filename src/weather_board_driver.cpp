#include <wiringPi.h>
#include <wiringPiI2C.h>

#include <iostream>
#include "ros/ros.h"

#include <weather_board_driver/wb_data.h>
#include <weather_board_driver/wb_list.h>

#include <stdio.h>
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

  int num_sensors = 0;
  n.getParam("/num_sensors", num_sensors);
  std::cout<<"number of sensors: "<<num_sensors<<std::endl;
  
  char *device;// stores the device path according to the devID
  int devID; // reads in ROS parameter integeter designating which device
  n.getParam("/device", devID);
  if (devID==1){
    device = "/dev/i2c-1";
  }
  else if (devID==2){
    device = "/dev/i2c-2";
  }
 
  std::cout<<"Device: "<<device<<std::endl;

  int lr;
  n.getParam("/hz", lr);
  std::cout<<"sensor frequency: "<<lr<<" hz"<<std::endl;
  ros::Rate loop_rate(lr);

  fd1 = wiringPiI2CSetupInterface(device, TCAADDR1);
  fd2 = wiringPiI2CSetupInterface(device, TCAADDR2);
  fd3 = wiringPiI2CSetupInterface(device, TCAADDR3);
  fd4 = wiringPiI2CSetupInterface(device, TCAADDR4);
  std::cout<<"starting multiplexer 0 at handle: "<<fd1<<std::endl;  
  std::cout<<"starting multiplexer 1 at handle: "<<fd2<<std::endl;
  std::cout<<"starting multiplexer 2 at handle: "<<fd3<<std::endl;
  std::cout<<"starting multiplexer 3 at handle: "<<fd4<<std::endl;  

  /*
  if (num_sensors>8){
    fd2 = wiringPiI2CSetupInterface(device, TCAADDR2);
    std::cout<<"starting multiplexer 1 at handle: "<<fd2<<std::endl;
    if (num_sensors>16){
      fd3 = wiringPiI2CSetupInterface(device, TCAADDR3);
      std::cout<<"starting multiplexer 2 at handle: "<<fd3<<std::endl;
      if (num_sensors>8){
	fd4 = wiringPiI2CSetupInterface(device, TCAADDR4);
	std::cout<<"starting multiplexer 3 at handle: "<<fd4<<std::endl;
      }
    }    
  }
  */

  for (int j=0; j<num_sensors;j++){
    tcaselect(j);
    std::cout<<"starting sensor number: "<<j<<std::endl;
    
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
  
    /*
    int com  = 5;//bme280_set_filter(BME280_FILTER_COEFF_OFF);
    int com1 = 5;//bme280_set_oversamp_humidity(BME280_OVERSAMP_1X);
    int com2 = 5;//bme280_set_power_mode(BME280_NORMAL_MODE);
    int com3 = 5;//bme280_set_standby_durn(BME280_STANDBY_TIME_1_MS);
    std::cout<<"com_rslt: "<<com<<", "<<com1<<", "<<com2<<", "<<com3<<std::endl;
  
    u_int8_t v_value;
    com = bme280_get_filter(&v_value);
    std::cout<<"com_rslt: "<<com<<std::endl;
    std::cout<<"v_value: "<<v_value<<std::endl;
    */

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
      
      // read in data
      data.UV_index = 0;//Si1132_readUV()/100.0;
      data.visible  = 0;//Si1132_readVisible();
      data.IR  = 0;//Si1132_readIR();
      bme280_read_pressure_temperature_humidity(&pressure, &temperature, &humidity);

      data.temperature  = temperature/100.0;
      data.humidity     = humidity/1024.0;
      data.pressure     = pressure/100.0;
      data.altitude     = bme280_readAltitude(pressure,SEALEVELPRESSURE_HPA);

      // push back into list
      list.wb_list.push_back(data);
    }    

    wb_pub.publish(list);

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
