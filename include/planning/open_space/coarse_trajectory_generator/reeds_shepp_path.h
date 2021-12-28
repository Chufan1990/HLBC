#pragma once

#include <memory>
#include <utility>
#include <vector>

#include "autoagric/common/vehicle_config.pb.h"
#include "autoagric/planning/planner_open_space_config.pb.h"
#include "planning/open_space/coarse_trajectory_generator/node3d.h"

namespace autoagric {
namespace planning {
struct ReedsSheppPath {
  std::vector<double> segs_lengths;
  std::vector<char> segs_types;
  double total_length = 0.0;
  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> phi;
  std::vector<bool> gear;
};

struct RSPParam {
  bool flag = false;
  double t = 0.0;
  double u = 0.0;
  double v = 0.0;
};

class ReedsShepp {
 public:
  ReedsShepp(const common::VehicleParam& vehicle_param,
             const PlannerOpenSpaceConfig& open_space_conf);

  virtual ~ReedsShepp() = default;

  bool ShortestRSP(const std::shared_ptr<Node3d> start_node,
                   const std::shared_ptr<Node3d> end_node,
                   std::shared_ptr<ReedsSheppPath> optimal_path);

 protected:
  bool GenerateRSPs(const std::shared_ptr<Node3d> start_node,
                    const std::shared_ptr<Node3d> end_node,
                    std::vector<ReedsSheppPath>* all_possible_path);

  bool GenerateRSP(const std::shared_ptr<Node3d> start_node,
                   const std::shared_ptr<Node3d> end_node,
                   std::vector<ReedsSheppPath>* all_possible_path);

  bool GenerateRSPPar(const std::shared_ptr<Node3d> start_node,
                      const std::shared_ptr<Node3d> end_node,
                      std::vector<ReedsSheppPath>* all_possible_path);

  bool GenerateLocalConfiguration(const std::shared_ptr<Node3d> start_node,
                                  const std::shared_ptr<Node3d> end_node,
                                  ReedsSheppPth* shortest_path);

  void Interpolation(const int index, const double pd, const char m,
                     const double ox, const double oy, const double ophi,
                     std::vector<double>* px, std::vector<double>* py,
                     std::vector<double>* pphi, std::vector<bool>* pgear);

  bool SetRSP(const int size, const double* lengths, const char* types,
              std::vector<ReedsSheppPath>* all_possible_paths);

  bool SetRSPPar(const int size, const double* lengths,
                 const std::string& types,
                 std::vector<ReedsSheppPath>* all_possible_paths,
                 const int idx);

  bool SCS(const double x, const double y, const double phi,
           std::vector<ReedsSheppPath>* all_possible_paths);
  bool CSC(const double x, const double y, const double phi,
           std::vector<ReedsSheppPath>* all_possible_paths);
  bool CCC(const double x, const double y, const double phi,
           std::vector<ReedsSheppPath>* all_possible_paths);
  bool CCCC(const double x, const double y, const double phi,
            std::vector<ReedsSheppPath>* all_possible_paths);
  bool CCSC(const double x, const double y, const double phi,
            std::vector<ReedsSheppPath>* all_possible_paths);
  bool CCSCC(const double x, const double y, const double phi,
             std::vector<ReedsSheppPath>* all_possible_paths);

  void LSL(const double x, const double y, const double phi, RSPParam* param);
  void LSR(const double x, const double y, const double phi, RSPParam* param);
  void LRL(const double x, const double y, const double phi, RSPParam* param);
  void SLS(const double x, const double y, const double phi, RSPParam* param);
  void LRLRn(const double x, const double y, const double phi, RSPParam* param);
  void LRLRp(const double x, const double y, const double phi, RSPParam* param);
  void LRSR(const double x, const double y, const double phi, RSPParam* param);
  void LRSL(const double x, const double y, const double phi, RSPParam* param);
  void LRSLR(const double x, const double y, const double phi, RSPParam* param);

  std::pair<double, double> calc_tau_omege(const double u, const double v,
                                           const double xi, const double eta,
                                           const double phi);

 protected:
  common::VehicleParam vehicle_param_;
  PlannerOpenSpaceConfig planner_open_space_config_;
  double max_kappa_;
};

}  // namespace planning
}  // namespace autoagric