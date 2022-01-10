# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include;/usr/include/eigen3".split(';') if "${prefix}/include;/usr/include/eigen3" != "" else []
PROJECT_CATKIN_DEPENDS = "geometry_msgs;visualization_msgs;roscpp;rospy;std_msgs;message_runtime;autoware_msgs".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lhlbc;/home/chufankong/autoware.ai/src/autoware/common/hlbc/third_party/protobuf/lib/libprotobuf.so;/usr/local/lib/libipopt.so;/usr/local/lib/libgflags.so.2.2.2".split(';') if "-lhlbc;/home/chufankong/autoware.ai/src/autoware/common/hlbc/third_party/protobuf/lib/libprotobuf.so;/usr/local/lib/libipopt.so;/usr/local/lib/libgflags.so.2.2.2" != "" else []
PROJECT_NAME = "hlbc"
PROJECT_SPACE_DIR = "/usr/local"
PROJECT_VERSION = "0.0.0"
