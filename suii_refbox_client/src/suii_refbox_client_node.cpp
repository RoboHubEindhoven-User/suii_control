#include <ros/ros.h>
#include <suii_refbox_client/suii_refbox_client.hpp>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "suii_refbox_client_node");
  ros::NodeHandle nh("~");
  ros::Rate loop_rate(10); // one Hz

  SuiiRefBoxClient refbox(nh);

  ROS_INFO("CFH Suii Robot is running!");

  while (ros::ok()) {
    refbox.sendBeacon();

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}
