#include "common/util/csv_reader.h"

#include <algorithm>

#include "common/macro.h"

namespace autoagric {
namespace common {
namespace util {

std::unordered_map<std::string, size_t> CSVReader::ParseHeaders(
    const char* filename) {
  std::ifstream ifs(filename);

  std::string header_line;
  std::getline(ifs, header_line);
  std::vector<std::string> header_names;

  ParseOneLine(header_line, &header_names);

  std::unordered_map<std::string, size_t> header_index_mapping;

  if (header_names.empty()) {
    AERROR("No header found");
    return header_index_mapping;
  }

  for (size_t i = 0; i < header_names.size(); i++) {
    header_index_mapping[header_names[i]] = i;
  }
  return header_index_mapping;
}

CSVReader::DataFormat CSVReader::CheckInput(
    const std::unordered_map<std::string, size_t>& headers) {
  auto x = headers.find("x");
  auto y = headers.find("y");
  auto t = headers.find("yaw");
  auto v = headers.find("velocity");
  auto k = headers.find("kappa");
  auto s = headers.find("s");

  if (x != end(headers) && y != end(headers) && t != end(headers)) {
    if (v != end(headers)) {
      if (k != end(headers)) {
        if (s != end(headers)) {
          return DataFormat::XYTVKS;
        }
        return DataFormat::XYTVK;
      }
      return DataFormat::XYTV;
    }
    return DataFormat::XYT;
  }

  return DataFormat::UNKNOWN;
}

bool CSVReader::LoadDataFromFile(const char* filename,
                                 StaticPathResult* result) {
  const auto headers = std::move(ParseHeaders(filename));

  const auto format = CheckInput(headers);

  if (CheckInput(headers) == DataFormat::UNKNOWN) {
    AERROR("Invalid data format. missing necessary attributes");
    return false;
  }
  
  std::ifstream ifs(filename);
  std::string line;

  std::getline(ifs, line);  // skip the header line;
  
  size_t index = 1;
  while (std::getline(ifs, line)) {
    if (!ParseContent(line, headers, format, result)) {
      AERROR("Error occurs when parse line " << index);
      return false;
    }
    index++;
  }
  return true;
}

bool CSVReader::ParseContent(
    const std::string& line,
    const std::unordered_map<std::string, size_t>& headers,
    const DataFormat format, StaticPathResult* result) {
  std::vector<std::string> contents;
  ParseOneLine(line, &contents);

  if (contents.size() != headers.size()) {
    AERROR("Data corrupted. Missing " << (headers.size() - contents.size())
                                      << " attribute(s)");
    return false;
  }

  result->x.push_back(std::stod(contents[headers.at("x")]));
  result->y.push_back(std::stod(contents[headers.at("y")]));
  result->phi.push_back(std::stod(contents[headers.at("yaw")]));

  if (format >= DataFormat::XYTV) {
    result->v.push_back(std::stod(contents[headers.at("velocity")]));
  }

  if (format >= DataFormat::XYTVK) {
    result->v.push_back(std::stod(contents[headers.at("kappa")]));
  }

  if (format >= DataFormat::XYTVKS) {
    result->v.push_back(std::stod(contents[headers.at("s")]));
  }

  return true;
}

void CSVReader::ParseOneLine(const std::string& line,
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

}  // namespace util
}  // namespace common
}  // namespace autoagric