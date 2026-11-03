#pragma once

#include <vector>

namespace argus::sitemap {

struct PoseNode {
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  bool fixed = false;
};

// A measurement of node `to` seen from node `from`, in `from`'s frame.
struct PoseEdge {
  int from = 0;
  int to = 0;
  double dx = 0.0;
  double dy = 0.0;
  double dyaw = 0.0;
  // Standard deviations, along x and y of `from` and of the turn.
  double sigma_m = 0.1;
  double sigma_rad = 0.05;
  // A robust edge can be switched off by the solver when it disagrees with the rest of the graph
  // (dynamic covariance scaling, Agarwal et al., ICRA 2013).
  bool robust = false;
};

struct PoseGraphResult {
  std::vector<PoseNode> nodes;
  // Weight each edge ended with: 1 is fully used, 0 is ignored. Always 1 for non-robust edges.
  std::vector<double> weights;
  int iterations = 0;
  double chi2 = 0.0;
  bool converged = false;
};

// Solves a 2D pose graph by damped Gauss-Newton on a sparse system. At least one node must be
// fixed. Throws std::invalid_argument on an edge naming a node that does not exist, on sigmas
// that are not positive, or with no fixed node.
PoseGraphResult optimize_pose_graph(const std::vector<PoseNode>& nodes, const std::vector<PoseEdge>& edges,
                                    int max_iterations = 50, double dcs_phi = 1.0);

}  // namespace argus::sitemap
