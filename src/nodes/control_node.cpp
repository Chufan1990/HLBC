#include "ros/ros.h"
#include "control/control_component.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "control");
  ros::NodeHandle nh("~");
  autoagric::control::ControlComponent control_component(nh);

  if (!control_component.Init()) return 1;
  if (!control_component.Proc()) return 1;

  return 0;
}