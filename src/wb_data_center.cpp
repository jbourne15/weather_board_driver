#include "ros/ros.h"
#include <iostream>
#include <sstream>
#include <string>
#include <unistd.h>
#include <vector>
#include <numeric>


#include <weather_board_driver/wb_data.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/Header.h>

std::vector<float> createTeam_data;

void createCallback(const weather_board_driver::wb_data msg){
    std::string ns = msg.header.frame_id;  
    int id = (int)ns[8]-'0'; // convert to in from char
    createTeam_data[id] = msg.humidity;
    //std::cout<<"id: "<<id<<" createTeam_data: "<<createTeam_data[id]<<std::endl;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "wb_data_center");
  ros::NodeHandle n;
  ros::Publisher team_pub = n.advertise<std_msgs::Float64MultiArray>("team_data", 1);

  std::vector<float> biasHlist;
  bool  biasFlag  = false; // flag designates when to publish wb data
  float biasMean  = 0;     // container for holding the humidity mean across sensors for many counts (biasNum)
  int   biasCount = 0;     // counter   
  int   biasNum   = 0;     // quantifies how many iterates to take average over
  n.getParam("/biasNum", biasNum);

  const std::string create = "/create";
  int numRobot;
  n.getParam("/numRobots", numRobot);
  ROS_INFO("num of robots: %i", numRobot);
  std::vector<ros::Subscriber> sub(numRobot);
  createTeam_data.resize(numRobot);
  std::fill(createTeam_data.begin(), createTeam_data.end(), -1);
  //  ros::Duration(5).sleep(); // let subscribers set

  std::vector<float> bias(numRobot,0); // init the bias correction
  
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

    if (biasFlag == true){
      // publish collected teamdata    
      std_msgs::Float64MultiArray arr;
      arr.layout.dim.push_back(std_msgs::MultiArrayDimension());
      arr.layout.dim[0].size = numRobot;
      arr.layout.dim[0].stride = 1;
      arr.layout.dim[0].label = "createTeamData";
    
      for (int i=0; i<numRobot; i++){	
	arr.data.push_back(createTeam_data[i]+bias[i]);
      }
      team_pub.publish(arr);
    }
    // initially gather humidity data to correct for the bias.
    else if (std::find(createTeam_data.begin(), createTeam_data.end(), -1) == createTeam_data.end()){
      for (int i=0; i<numRobot; i++){
	biasHlist.push_back(createTeam_data[i]);	
      }

      if (biasCount+1>=biasNum){//check if we have enough data 
	for (int k=0; k<biasHlist.size();k++){
	}
	
	double sum = std::accumulate(biasHlist.begin(), biasHlist.end(), 0.0);
	if (!biasHlist.empty()){
	  biasMean = sum / biasHlist.size();
	}
	biasFlag = true;
	if (biasNum!=0){std::cout<<"bias has been corrected"<<std::endl;}

	if (biasNum!=0){ // require biasNum>0 else bias is 0 forall sensors (initialized)
	  for (int j=0; j<numRobot;j++){	  
	    bias[j] = biasMean - biasHlist[j];
	    std::cout<<"j: "<<j<<", bias: "<<bias[j]<<" biasHlist: "<<biasHlist[j]<<std::endl;
	  }
	}
      }
      biasCount+=1;
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
