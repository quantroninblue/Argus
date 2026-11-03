#include "argus/sitemap/pose_graph.hpp"

#include <Eigen/Sparse>
#include <Eigen/SparseCholesky>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace argus::sitemap {

namespace {

double wrap(double a) { return std::atan2(std::sin(a), std::cos(a)); }

using Vec3d = Eigen::Vector3d;
using Mat3d = Eigen::Matrix3d;

// The error of an edge, in the measurement's frame (Grisetti et al., A Tutorial on Graph-Based
// SLAM, 2010), with its Jacobians for the two nodes.
Vec3d edge_error(const PoseNode& a, const PoseNode& b, const PoseEdge& e, Mat3d* Ja, Mat3d* Jb) {
  const double ca = std::cos(a.yaw), sa = std::sin(a.yaw);
  const double cz = std::cos(e.dyaw), sz = std::sin(e.dyaw);
  const double tx = b.x - a.x, ty = b.y - a.y;
  // Node b in a's frame.
  const double lx = ca * tx + sa * ty;
  const double ly = -sa * tx + ca * ty;
  const double rx = lx - e.dx, ry = ly - e.dy;
  Vec3d err(cz * rx + sz * ry, -sz * rx + cz * ry, wrap(b.yaw - a.yaw - e.dyaw));
  if (Ja != nullptr) {
    const double dlx = -sa * tx + ca * ty, dly = -ca * tx - sa * ty;
    Mat3d Ra;
    Ra << -ca, -sa, dlx, sa, -ca, dly, 0.0, 0.0, -1.0;
    Mat3d Rb;
    Rb << ca, sa, 0.0, -sa, ca, 0.0, 0.0, 0.0, 1.0;
    Mat3d Z;
    Z << cz, sz, 0.0, -sz, cz, 0.0, 0.0, 0.0, 1.0;
    *Ja = Z * Ra;
    *Jb = Z * Rb;
  }
  return err;
}

double edge_chi2(const Vec3d& err, const PoseEdge& e) {
  const double w_t = 1.0 / (e.sigma_m * e.sigma_m);
  const double w_r = 1.0 / (e.sigma_rad * e.sigma_rad);
  return w_t * (err.x() * err.x() + err.y() * err.y()) + w_r * err.z() * err.z();
}

double dcs_scale(double chi2, double phi) { return std::min(1.0, 2.0 * phi / (phi + chi2)); }

double total_chi2(const std::vector<PoseNode>& nodes, const std::vector<PoseEdge>& edges, double phi) {
  double total = 0.0;
  for (const PoseEdge& e : edges) {
    const double c = edge_chi2(edge_error(nodes[e.from], nodes[e.to], e, nullptr, nullptr), e);
    if (e.robust) {
      const double s = dcs_scale(c, phi);
      total += s * s * c + phi * (s - 1.0) * (s - 1.0);
    } else {
      total += c;
    }
  }
  return total;
}

// Damped Gauss-Newton at one phi. Returns false if it ran out of iterations.
bool settle(PoseGraphResult& out, const std::vector<PoseEdge>& edges, const std::vector<int>& column, int unknowns,
            int n, int max_iterations, double dcs_phi) {
  double damping = 1e-6;
  for (int iteration = 0; iteration < max_iterations; ++iteration, ++out.iterations) {
    std::vector<Eigen::Triplet<double>> triplets;
    triplets.reserve(edges.size() * 36 + static_cast<size_t>(unknowns));
    Eigen::VectorXd gradient = Eigen::VectorXd::Zero(unknowns);
    for (const PoseEdge& e : edges) {
      Mat3d Ja, Jb;
      const Vec3d err = edge_error(out.nodes[e.from], out.nodes[e.to], e, &Ja, &Jb);
      Vec3d info(1.0 / (e.sigma_m * e.sigma_m), 1.0 / (e.sigma_m * e.sigma_m), 1.0 / (e.sigma_rad * e.sigma_rad));
      if (e.robust) {
        const double s = dcs_scale(edge_chi2(err, e), dcs_phi);
        info *= s * s;
      }
      const Mat3d omega = info.asDiagonal();
      const int ca = column[e.from], cb = column[e.to];
      const auto add = [&triplets](int row, int col, const Mat3d& block) {
        for (int r = 0; r < 3; ++r) {
          for (int c = 0; c < 3; ++c) {
            triplets.emplace_back(row + r, col + c, block(r, c));
          }
        }
      };
      if (ca >= 0) {
        gradient.segment<3>(ca) += Ja.transpose() * omega * err;
        add(ca, ca, Ja.transpose() * omega * Ja);
      }
      if (cb >= 0) {
        gradient.segment<3>(cb) += Jb.transpose() * omega * err;
        add(cb, cb, Jb.transpose() * omega * Jb);
      }
      if (ca >= 0 && cb >= 0) {
        add(ca, cb, Ja.transpose() * omega * Jb);
        add(cb, ca, Jb.transpose() * omega * Ja);
      }
    }

    // Damp until a step lowers the cost, so that a robust edge turning off cannot make it diverge.
    bool improved = false;
    const double before = total_chi2(out.nodes, edges, dcs_phi);
    double step_size = 0.0;
    for (int attempt = 0; attempt < 12 && !improved; ++attempt) {
      std::vector<Eigen::Triplet<double>> damped = triplets;
      for (int k = 0; k < unknowns; ++k) {
        damped.emplace_back(k, k, damping);
      }
      Eigen::SparseMatrix<double> H(unknowns, unknowns);
      H.setFromTriplets(damped.begin(), damped.end());
      Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver(H);
      if (solver.info() == Eigen::Success) {
        const Eigen::VectorXd step = solver.solve(-gradient);
        if (solver.info() == Eigen::Success && step.allFinite()) {
          std::vector<PoseNode> trial = out.nodes;
          for (int i = 0; i < n; ++i) {
            if (column[i] >= 0) {
              trial[i].x += step(column[i]);
              trial[i].y += step(column[i] + 1);
              trial[i].yaw = wrap(trial[i].yaw + step(column[i] + 2));
            }
          }
          if (total_chi2(trial, edges, dcs_phi) <= before) {
            out.nodes = std::move(trial);
            step_size = step.lpNorm<Eigen::Infinity>();
            improved = true;
            damping = std::max(1e-9, damping * 0.3);
            break;
          }
        }
      }
      damping *= 10.0;
    }
    if (!improved || step_size < 1e-7) {
      return true;
    }
  }
  return false;
}

}  // namespace

PoseGraphResult optimize_pose_graph(const std::vector<PoseNode>& nodes, const std::vector<PoseEdge>& edges,
                                    int max_iterations, double dcs_phi) {
  const int n = static_cast<int>(nodes.size());
  std::vector<int> column(nodes.size(), -1);
  int unknowns = 0;
  for (int i = 0; i < n; ++i) {
    if (!nodes[i].fixed) {
      column[i] = unknowns;
      unknowns += 3;
    }
  }
  if (unknowns == 3 * n && n > 0) {
    throw std::invalid_argument("a pose graph needs a fixed node");
  }
  for (const PoseEdge& e : edges) {
    if (e.from < 0 || e.from >= n || e.to < 0 || e.to >= n || e.from == e.to) {
      throw std::invalid_argument("pose graph edge names a node that does not exist");
    }
    if (!(e.sigma_m > 0.0) || !(e.sigma_rad > 0.0)) {
      throw std::invalid_argument("pose graph edge sigmas must be positive");
    }
  }

  PoseGraphResult out;
  out.nodes = nodes;
  out.weights.assign(edges.size(), 1.0);
  out.chi2 = total_chi2(out.nodes, edges, dcs_phi);
  if (unknowns == 0) {
    out.converged = true;
    return out;
  }

  // A true loop edge starts with a large error, which at the final phi would switch it off before
  // it could pull anything. So every edge starts trusted and phi tightens: an edge that fits the
  // rest ends with a small error and stays, and one that does not is switched off at the end.
  out.converged = true;
  for (const double phi : {dcs_phi * 1e4, dcs_phi * 1e2, dcs_phi}) {
    out.converged = settle(out, edges, column, unknowns, n, max_iterations, phi) && out.converged;
  }

  for (size_t k = 0; k < edges.size(); ++k) {
    if (edges[k].robust) {
      const PoseEdge& e = edges[k];
      out.weights[k] = dcs_scale(edge_chi2(edge_error(out.nodes[e.from], out.nodes[e.to], e, nullptr, nullptr), e),
                                 dcs_phi);
    }
  }
  out.chi2 = total_chi2(out.nodes, edges, dcs_phi);
  return out;
}

}  // namespace argus::sitemap
