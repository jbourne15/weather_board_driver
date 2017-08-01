#include "ros/ros.h"
#include <iostream>
#include <sstream>
#include <string>
#include <unistd.h>

#include <weather_board_driver/wb_data.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/Header.h>

std::vector<float> createTeam_data;

void createCallback(const weather_board_driver::wb_data msg){
  if (!createTeam_data.empty()){
    std::string ns = msg.header.frame_id;  
    int id = (int)ns[8]-'0'; // convert to in from char
    createTeam_data[id] = msg.humidity;
  }
  else {
    sleep(2);// wait until createTeam_data has size numRobot
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "wb_data_center");
  ros::NodeHandle n;
  ros::Publisher team_pub = n.advertise<std_msgs::Float64MultiArray>("team_data", 1);

  const std::string create = "/create";
  int numRobot;
  n.getParam("/numRobots", numRobot);
  ROS_INFO("num of robots: %i", numRobot);
  std::vector<ros::Subscriber> sub(numRobot);
  createTeam_data.resize(numRobot);
  
  //setup subscribers
  std::string createTopic;
  for (int i=0; i<numRobot; i++){
    std::ostringstream ss;
    ss << i;
    createTopic = create+ss.str()+"/wb_data"; //abide by the namescape
    sub[i] = n.subscribe(createTopic, 1, createCallback);
  }

  int lr;
  n.getParam("/datacenterhz", lr);
  ros::Rate loop_rate(lr);
  ROS_INFO("datacenter frequency: %i", lr);

  while (ros::ok()) {

    // publish collected teamdata    
    std_msgs::Float64MultiArray arr;
    arr.layout.dim.push_back(std_msgs::MultiArrayDimension());
    arr.layout.dim[0].size = numRobot;
    arr.layout.dim[0].stride = 1;
    arr.layout.dim[0].label = "createTeamData";
    
    for (int i=0; i<numRobot; i++){
      arr.data.push_back(createTeam_data[i]);
    }
    
    team_pub.publish(arr);

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
