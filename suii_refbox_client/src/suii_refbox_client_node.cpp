/************** Copyright (c) 2019, RoboHubEindhoven. All rights reserved **************
 * @Project     : suii_refbox_client
 * @file        : suii_refbox_client_node.cpp
 *
 * @brief       : This file cummunicates with the robocup@work refbox central factory hub 
 *                and the suii_task_manager.
 *
 * @author      : Thierry Zinkeng
 * Contact:     : thi.zinkeng@gmail.com
 *
 ***************************************************************************************/
#include <ros/ros.h>
#include <suii_refbox_client/suii_refbox_client.hpp>

int main(int argc, char **argv)
{
  std::cout<<"Starting Suii Refbox Client"<<std::endl;

  ros::init(argc, argv, "suii_refbox_client_node");
  ros::NodeHandle nh("~");
  ros::Rate loop_rate(10); // one Hz

  SuiiRefBoxClient refbox(nh);

  ROS_INFO("Suii Refbox Client initialized and running!");

  while (ros::ok()) {
    refbox.sendBeacon();

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}
