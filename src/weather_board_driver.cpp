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

#define TCAADDR 0x70 // same for both multiplexers (on different i2c buses)

// file handlers for the multiplexers
int fd1; 
int fd2;

// file handlers for the sensor of the 1st and 2nd multiplexer
int bme280Fd1;
int bme280Fd2;

void tcaselect(int i, int fd){
  if (i>7){
    wiringPiI2CWrite(fd,1<<(i-8));
  }
  else{
    wiringPiI2CWrite(fd,1<<i);
  }
}

int main(int argc, char **argv)
{
  char *device1 = "/dev/i2c-1";
  char *device2 = "/dev/i2c-2";
  char *device; // temp container
  
  ros::init(argc, argv, "weather_board_driver");
  ros::NodeHandle n;
  ros::Publisher wb_pub = n.advertise<weather_board_driver::wb_list>("wb_list", 1);

  int num_sensors = 0;
  n.getParam("/num_sensors", num_sensors);
  std::cout<<"number of sensors: "<<num_sensors<<std::endl;

  int lr;
  n.getParam("/hz", lr);
  std::cout<<"sensor frequency: "<<lr<<" hz"<<std::endl;
  ros::Rate loop_rate(lr);

  fd1 = wiringPiI2CSetupInterface(device1, TCAADDR);
  std::cout<<"1st multiplexer handle: "<<fd1<<std::endl;

  if (num_sensors>8){
    fd2 = wiringPiI2CSetupInterface(device2, TCAADDR);
    std::cout<<"2nd multiplexer handle: "<<fd2<<std::endl;
  }

  for (int j=0; j<num_sensors;j++){
 
    if (j>7){
      tcaselect(j, fd2);
      std::cout<<"starting the "<<j<< "th bme280 sensor on fd1: "<<fd2<<" handle"<<std::endl;
      device = device2;
    }
    else{
      tcaselect(j, fd1);
      std::cout<<"starting the "<<j<< "th bme280 sensor on fd: "<<fd1<<" handle"<<std::endl;
      device = device1;
    }    
    
    //si1132_begin(device);
    if (bme280_begin(device) < 0) {
      std::cerr<<"bme280 failed on sensor: "<<j<<std::endl;
      return -1;
    }
    // save so I can update bme280Fd global var later in the while loop
    if (j<7){bme280Fd1 = bme280Fd;} // same for a specific multiplexer (dependent on the i2c bus)
    else{    bme280Fd2 = bme280Fd;}
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

      if (j>7){//select which sensor from multiplexer
	tcaselect(j, fd2);
	bme280Fd = bme280Fd2; //reset global var in bme280 api
      }
      else{
	tcaselect(j, fd1);
	bme280Fd = bme280Fd1; //reset global var in bme280 api
      }

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
