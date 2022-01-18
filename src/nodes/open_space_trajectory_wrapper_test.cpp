#include "planning/tasks/optimizers/open_space_trajectory_generation/open_space_trajectory_wrapper.h"

#include "common/macro.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "open_space");
  ros::NodeHandle nh("~");

  autoagric::planning::OpenSpaceTrajectoryWrapper open_space_trajectory_wrapper(
      nh);

  if (!open_space_trajectory_wrapper.Init()) {
    AERROR("OpenSpaceTrajectoryWrapper initialization failed");
    return 1;
  }

  if (!open_space_trajectory_wrapper.Proc()) {
    AERROR("OpenSpaceTrajectoryWrapper process failed");
    return 1;
  }

  return 0;
}