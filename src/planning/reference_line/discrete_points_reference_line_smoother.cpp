// #include "planning/reference_line/discrete_points_reference_line_smoother.h"

// #include <algorithm>

// #include "common/macro.h"
// #include "planning/math/discrete_points_math.h"
// #include "planning/math/discretized_points_smoothing/cos_theta_smoother.h"

// namespace autoagric {
// namespace planning {
// DiscretePointsReferenceLineSmoother::DiscretePointsReferenceLineSmoother(
//     const ReferenceLineSmootherConfig& config)
//     : ReferenceLineSmoother(config) {}

// bool DiscretePointsReferenceLineSmoother::Smooth(
//     const ReferenceLine& raw_reference_line,
//     ReferenceLine* const smoothed_reference_line) {
//   std::vector<std::pair<double, double>> raw_point2d;
//   std::vector<double> anchorpoints_lateralbound;

//   for (const auto& anchor_point : anchor_points_) {
//     raw_point2d.emplace_back(anchor_point.path_point.x(),
//                              anchor_point.path_point.y());
//     anchorpoints_lateralbound.emplace_back(anchor_point.lateral_bound);
//   }

//   anchorpoints_lateralbound.front() = 0.0;
//   anchorpoints_lateralbound.back() = 0.0;

//   NormalizePoints(&raw_point2d);

//   bool status = false;

//   const auto& smoothing_method = config_.discrete_points().smoothing_method();

//   std::vector<std::pair<double, double>> smoothed_point2d;

//   //   switch (smoothing_method) {
//   //     case DiscretePointSmootherConfig::COS_THETA_SMOOTHING:
//   //       status = CosThetaSmooth(raw_point2d, anchorpoints_lateralbound,
//   //                               &smoothed_point2d);
//   //       break;
//   //     case DiscretePointSmootherConfig::FEM_POS_DEVIATION_SMOOTHING:
//   //       status = FemPosSmooth(raw_point2d, anchorpoints_lateralbound,
//   //                             &smoothed_point2d);
//   //       break;
//   //     default:
//   //       AERROR("", "smoother type not defined");
//   //       break;
//   //   }

//   /**
//    * @note force to use costheta smoother
//    */
//   status =
//       CosThetaSmooth(raw_point2d, anchorpoints_lateralbound, &smoothed_point2d);

//   if (!status) {
//     AERROR("", "discrete_points reference line smoother failed");
//     return false;
//   }

//   DeNormalizePoints(&smoothed_point2d);

//   std::vector<ReferencePoint> ref_points;
//   GenerateRefPointProfile(raw_reference_line, smoothed_point2d, &ref_points);

//   ReferencePoint::RemoveDuplicates(&ref_points);

//   if (ref_points.size() < 2) {
//     AERROR("", "fail to generate smoothed reference line.");
//     return false;
//   }

//   *smoothed_reference_line = ReferenceLine(&ref_points);

//   return true;
// }

// bool DiscretePointsReferenceLineSmoother::CosThetaSmooth(
//     const std::vector<std::pair<double, double>>& raw_point2d,
//     const std::vector<double>& bounds,
//     std::vector<std::pair<double, double>>* ptr_smoothed_point2d) {
//   const auto& cos_theta_config =
//       config_.discrete_points().cos_theta_smoothing();

//   CosThetaSmoother smoother(cos_theta_config);

//   // box contraints on pos are used in cos theta smoother, thus shrink the
//   // bounds by 1.0 / sqrt(2.0)
//   std::vector<double> box_bounds = bounds;
//   const double box_ratio = 1.0 / std::sqrt(2.0);
//   for (auto& bound : box_bounds) {
//     bound *= box_ratio;
//   }

//   std::vector<double> opt_x;
//   std::vector<double> opt_y;
//   bool status = smoother.Solve(raw_point2d, box_bounds, &opt_x, &opt_y);

//   if (!status) {
//     AERROR("", "costheta reference line smoothing failed");
//     return false;
//   }

//   if (opt_x.size() < 2 || opt_y.size() < 2) {
//     AERROR("", "return by costheta smoother is wrong. size smaller than 2");
//     return false;
//   }

//   CHECK_EQ(opt_x.size(), opt_y.size()) << "x and y result size not equal";

//   std::size_t point_size = opt_x.size();
//   for (std::size_t i = 0; i < point_size; i++) {
//     ptr_smoothed_point2d->emplace_back(opt_x[i], opt_y[i]);
//   }

//   return true;
// }

// void DiscretePointsReferenceLineSmoother::SetAnchorPoints(
//     const std::vector<AnchorPoint>& anchor_points) {
//   CHECK_GT(anchor_points.size(), 1U);
//   anchor_points_ = anchor_points;
// }

// void DiscretePointsReferenceLineSmoother::NormalizePoints(
//     std::vector<std::pair<double, double>>* xy_points) {
//   zero_x_ = xy_points->front().first;
//   zero_y_ = xy_points->front().second;

//   std::for_each(xy_points->begin(), xy_points->end(),
//                 [this](std::pair<double, double>& point) {
//                   auto curr_x = point.first;
//                   auto curr_y = point.second;
//                   std::pair<double, double> xy(curr_x - zero_x_,
//                                                curr_y - zero_y_);
//                   point = std::move(xy);
//                 });
// }

// void DiscretePointsReferenceLineSmoother::DeNormalizePoints(
//     std::vector<std::pair<double, double>>* xy_points) {
//   std::for_each(xy_points->begin(), xy_points->end(),
//                 [this](std::pair<double, double>& point) {
//                   auto curr_x = point.first;
//                   auto curr_y = point.second;
//                   std::pair<double, double> xy(curr_x + zero_x_,
//                                                curr_y + zero_y_);
//                   point = std::move(xy);
//                 });
// }

// bool DiscretePointsReferenceLineSmoother::GenerateRefPointProfile(
//     const ReferenceLine& raw_reference_line,
//     const std::vector<std::pair<double, double>>& xy_points,
//     std::vector<ReferencePoint>* reference_points) {
//   std::vector<double> headings;
//   std::vector<double> kappas;
//   std::vector<double> dkappas;
//   std::vector<double> accumulated_s;

//   if (!DiscretePointsMath::ComputePathPofile(
//           xy_points, &headings, &accumulated_s, &kappas, &dkappas)) {
//     return false;
//   }
//   // Load into ReferencePoints
//   // size_t points_size = xy_points.size();
//   // for (size_t i = 0; i < points_size; ++i) {
//   //   common::SLPoint ref_sl_point;
//   //   if (!raw_reference_line.XYToSL({xy_points[i].first, xy_points[i].second},
//   //                                  &ref_sl_point)) {
//   //     return false;
//   //   }
//   //   const double kEpsilon = 1e-6;
//   //   if (ref_sl_point.s() < -kEpsilon ||
//   //       ref_sl_point.s() > raw_reference_line.Length()) {
//   //     continue;
//   //   }
//   //   ref_sl_point.set_s(std::max(ref_sl_point.s(), 0.0));
//   //   ReferencePoint rlp = raw_reference_line.GetReferencePoint(ref_sl_point.s());
//   //   auto new_lane_waypoints = rlp.lane_waypoints();
//   //   for (auto& lane_waypoint : new_lane_waypoints) {
//   //     lane_waypoint.l = ref_sl_point.l();
//   //   }
//   //   reference_points->emplace_back(ReferencePoint(
//   //       hdmap::MapPathPoint(
//   //           common::math::Vec2d(xy_points[i].first, xy_points[i].second),
//   //           headings[i], new_lane_waypoints),
//   //       kappas[i], dkappas[i]));
//   // }
// }

// }  // namespace planning
// }  // namespace autoagric