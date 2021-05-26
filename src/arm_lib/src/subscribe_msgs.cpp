#include <ros/ros.h>
#include "arm_lib/msg_collection.h"

void publishMsg_callback(const arm_lib::msg_collection &subscribeMsg)
{
  std::cout << "Rotated and Translated\n"
            << "x' = " << subscribeMsg.newX << "\n"
            << "y' = " << subscribeMsg.newY << "\n"
            << "z' = " << subscribeMsg.newZ << std::endl;

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "vectorSubscribe");
  ros::NodeHandle node;

  ros::Subscriber subscribe_color = node.subscribe("/publishVector", 1000, publishMsg_callback);

  ros::spin();

  return 0;
}