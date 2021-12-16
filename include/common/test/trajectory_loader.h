#pragma once

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include "common/macro.h"
#include "hlbc/PathPoint.h"
#include "hlbc/Trajectory.h"
#include "hlbc/TrajectoryPoint.h"

namespace autoagric {
namespace common {
namespace test {

class TrajectoryLoader {
 public:
  enum class Format : int32_t {
    POSITION = 1,  // x,y,z,(velocity)
    POSE = 2,      // x,y,z,yaw,(velocity)
    HEADER = 3,    // first line consists on explanation of values
    UNKNOWN = 0,
  };

  template <typename T, typename P>
  void Load(const std::string& file_path, T* lane) {
    if (!VerifyFileConsistency(file_path.c_str())) {
      AERROR("Verifying given file failed");
      return;
    }

    AINFO("Data is valid");
    std::vector<P> vec;
    LoadTrajectoryFromFile<P>(file_path.c_str(), &vec);

    Vector2Trajectory(vec, lane);
  }

 private:
  template <typename T>
  void LoadTrajectoryFromFile(const char* filename, std::vector<T>* lane) {
    std::ifstream ifs(filename);

    if (!ifs) {
      return;
    }

    std::string line;
    std::getline(ifs, line);  // get first line
    std::vector<std::string> contents;
    ParseOneLine(line, &contents);

    // std::getline(ifs, line);  // remove second line
    while (std::getline(ifs, line)) {
      T p;
      Parse<T>(line, contents, &p);
      lane->emplace_back(p);
    }
  }

  std::vector<std::string> Header(const char* filename);

  template <typename T>
  void Parse(const std::string& line, const std::vector<std::string>& contents,
             T* p) {
    std::vector<std::string> columns;
    ParseOneLine(line, &columns);
    std::unordered_map<std::string, std::string> map;
    for (size_t i = 0; i < contents.size(); i++) {
      map[contents.at(i)] = columns.at(i);
    }

    const double x = std::stod(map["x"]);
    const double y = std::stod(map["y"]);
    const double theta = std::stod(map["yaw"]);
    const double speed = std::stod(map["velocity"]);
    const double kappa = std::stod(map["kappa"]);
    const double station = std::stod(map["s"]);
    ToPoint(x, y, theta, speed, kappa, station, p);
  }

  void ToPoint(const double x, const double y, const double theta,
               const double speed, const double kappa, const double station,
               hlbc::TrajectoryPoint* p);

  bool VerifyFileConsistency(const char* filename);

  Format CheckFileFormat(const char* filename);

  void ParseOneLine(const std::string& line, std::vector<std::string>* columns);

  size_t Cols(const std::string& line);

  void Vector2Trajectory(std::vector<hlbc::TrajectoryPoint>& vec,
                         hlbc::Trajectory* traj);

  size_t num_of_attributes_ = 0;

  std::vector<std::string> headers_;
};
}  // namespace test

}  // namespace common
}  // namespace autoagric
