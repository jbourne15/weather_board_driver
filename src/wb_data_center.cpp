#include "ros/ros.h"
#include <iostream>
#include <sstream>
#include <string>
//#include "boost/bind.hpp"

#include <weather_board_driver/wb_data.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/Header.h>
      
void createCallback(const weather_board_driver::wb_data::ConstPtr& msg){
  //msg.header.seq stands for the specific robot create ID
  //createTeam_data[msg.header.seq] = msg.humidity  
  //std_msgs::Header data = msg.header;
  //float data = msg;
  //ROS_INFO("data: %i, %f", msg.seq, data);
  std::cout<<msg<<std::endl;
  //ROS_INFO(msg);
  return;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "wb_data_center");
  ros::NodeHandle n;
  ros::Publisher team_pub = n.advertise<std_msgs::Float64MultiArray>("createTeam_data", 1);

  //Subscriber
  const std::string create = "/create";
  unsigned int numRobots = 1;
  //n.getParam("/numRobots", numRobots);
  std::vector<ros::Subscriber> sub(numRobots);
  std::vector<float> createTeam_data(numRobots);
  
  //setup subscribers
  std::string createTopic;
  for (int i=0; i<numRobots; i++){
    std::ostringstream ss;
    ss << i;
    createTopic = create+ss.str()+"wb_data"; //abide by the namescape
    //createTopic = create+std::to_string(i)+"wb_data"; //abide by the namescape
    sub[i] = n.subscribe(createTopic, 1, createCallback);
  }

  int lr;
  //n.getParam("/hz", lr);
  ros::Rate loop_rate(50);

  while (ros::ok()) {

    // publish collected teamdata    
    std_msgs::Float64MultiArray arr;
    arr.layout.dim.push_back(std_msgs::MultiArrayDimension());
    arr.layout.dim[0].size = numRobots;
    arr.layout.dim[0].stride = 1;
    arr.layout.dim[0].label = "createTeamData";
    
    for (int i=0; i<numRobots; i++){
      arr.data.push_back(createTeam_data[i]);
    }
    
    team_pub.publish(arr);

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
