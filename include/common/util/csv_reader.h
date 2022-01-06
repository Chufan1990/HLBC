#pragma once

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoagric {
namespace common {
namespace util {

struct StaticPathResult {
  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> phi;
  std::vector<double> v;
  std::vector<double> a;
  std::vector<double> kappa;
  std::vector<double> dkappa;
  std::vector<double> accumulated_s;
  std::vector<double> relative_time;
  std::vector<bool> gear;  // true: forward, false: backward
  
};

class CSVReader {
 public:
  enum class DataFormat : uint8_t {
    UNKNOWN = 0,
    XYT = 1,
    XYTV = 2,
    XYTVK = 3,
    XYTVKS = 4
  };

  bool LoadDataFromFile(const char* filename, StaticPathResult* result);

 private:
  std::unordered_map<std::string, size_t> ParseHeaders(const char* filename);

  DataFormat CheckInput(const std::unordered_map<std::string, size_t>& headers);

  bool ParseContent(const std::string& line,
                    const std::unordered_map<std::string, size_t>& headers,
                    const DataFormat format, StaticPathResult* result);

  void ParseOneLine(const std::string& line, std::vector<std::string>* columns);
};
}  // namespace util

}  // namespace common
}  // namespace autoagric
