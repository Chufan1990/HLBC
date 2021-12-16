#include "common/test/trajectory_loader.h"

#include <algorithm>

namespace autoagric {

namespace common {
namespace test {

using hlbc::PathPoint;
using hlbc::Trajectory;
using hlbc::TrajectoryPoint;

bool TrajectoryLoader::VerifyFileConsistency(const char* filename) {
  AINFO("Verifing...");
  std::ifstream ifs(filename);

  if (!ifs) {
    AERROR("Cannot find specified file " << filename);
    return false;
  }

  Format format = CheckFileFormat(filename);
  if (format == Format::UNKNOWN) {
    AERROR("Unknown format");
    return false;
  }

  num_of_attributes_ = headers_.size();

  std::string line;

  while (std::getline(ifs, line))  // search from second line
  {
    if (Cols(line) != num_of_attributes_) {
      AERROR("Data incomplete. Attributes missing");
      return false;
    }
  }
  AINFO("Verification done");
  return true;
}

TrajectoryLoader::Format TrajectoryLoader::CheckFileFormat(
    const char* filename) {
  headers_ = Header(filename);

  if (std::find(begin(headers_), end(headers_), "x") == end(headers_)) {
    AERROR("Given data doesn't have x attribute");
    return Format::UNKNOWN;
  }

  if (std::find(begin(headers_), end(headers_), "y") == end(headers_)) {
    AERROR("Given data doesn't have y attribute");
    return Format::UNKNOWN;
  }

  if (std::find(begin(headers_), end(headers_), "velocity") == end(headers_)) {
    AERROR("Given data doesn't have velocity attribute");
    return Format::UNKNOWN;
  }

  if (std::find(begin(headers_), end(headers_), "s") == end(headers_)) {
    AWARN("Given data doesn't have station attribute");
  }

  if (std::find(begin(headers_), end(headers_), "kappa") == end(headers_)) {
    AWARN("Given data doesn't have curvature attribute");
  }

  if (std::find(begin(headers_), end(headers_), "relative_time") ==
      end(headers_)) {
    AWARN("Given data doesn't have relative time attribute");
  }

  return Format::HEADER;
}

std::vector<std::string> TrajectoryLoader::Header(const char* filename) {
  std::ifstream ifs(filename);

  std::string header_line;
  std::getline(ifs, header_line);
  std::vector<std::string> headers;

  ParseOneLine(header_line, &headers);

  if (headers.empty()) {
    AERROR("No header found");
  }
  return headers;
}

void TrajectoryLoader::ParseOneLine(const std::string& line,
                                    std::vector<std::string>* columns) {
  std::istringstream ss(line);
  std::string column;
  while (std::getline(ss, column, ',')) {
    auto res = std::find(column.begin(), column.end(), ' ');
    while (res != column.end()) {
      column.erase(res);
      res = std::find(column.begin(), column.end(), ' ');
    }
    res = std::find(column.begin(), column.end(), '\n');
    while (res != column.end()) {
      column.erase(res);
      res = std::find(column.begin(), column.end(), '\n');
    }
    res = std::find(column.begin(), column.end(), '\r');
    while (res != column.end()) {
      column.erase(res);
      res = std::find(column.begin(), column.end(), '\r');
    }

    if (!column.empty()) {
      columns->emplace_back(column);
    }
  }
}

size_t TrajectoryLoader::Cols(const std::string& line) {
  std::istringstream ss(line);
  size_t ncol = 0;

  std::string column;
  while (std::getline(ss, column, ',')) {
    ncol++;
  }
  return ncol;
}

void TrajectoryLoader::ToPoint(const double x, const double y,
                               const double theta, const double speed,
                               const double kappa, const double station,
                               TrajectoryPoint* p) {
  p->path_point.x = x;
  p->path_point.y = y;
  p->path_point.theta = theta;
  p->path_point.s = station;
  p->v = speed;
  p->path_point.kappa = kappa;
}

void TrajectoryLoader::Vector2Trajectory(std::vector<TrajectoryPoint>& vec,
                                         Trajectory* traj) {
  traj->trajectory_point = std::move(vec);
}

}  // namespace test
}  // namespace common
}  // namespace autoagric