#include "common/macro.h"
#include "planning/open_space/tools/iterative_anchoring_smoothing_wrapper.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "open_space");
  ros::NodeHandle nh("~");

  autoagric::planning::IterativeAnchoringSmoothingWrapper
      iterative_anchoring_smoothing_wrapper(nh);

  if (!iterative_anchoring_smoothing_wrapper.Init()) {
    AERROR("iterative anchoring smoothing wrapper initialization failed");
    return 1;
  }

  AINFO("iterative anchoring smoothing wrapper initialization done");

  if (!iterative_anchoring_smoothing_wrapper.Proc()) {
    AERROR("iterative anchoring smoothing wrapper  process failed");
    return 1;
  }

  return 0;
}