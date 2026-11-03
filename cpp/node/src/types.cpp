#include "argus/node/types.hpp"

#include <cmath>

namespace argus::node {

Eigen::Isometry3d Pose4::transform() const {
  Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
  t.translate(position);
  t.rotate(Eigen::AngleAxisd(yaw, Vec3::UnitZ()));
  return t;
}

Vec3 Pose4::rotate(const Vec3& v) const {
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  return {c * v.x() - s * v.y(), s * v.x() + c * v.y(), v.z()};
}

Vec3 Pose4::to_odom(const Vec3& p) const { return position + rotate(p); }

double wrap_angle(double a) { return std::remainder(a, 2.0 * M_PI); }

std::string to_string(FailureCause cause) {
  switch (cause) {
    case FailureCause::None:
      return "none";
    case FailureCause::DeadEndObserved:
      return "dead_end_observed";
    case FailureCause::SearchedNoGoal:
      return "searched_no_goal";
    case FailureCause::TemporaryObstruction:
      return "temporary_obstruction";
    case FailureCause::ControllerNoProgress:
      return "controller_no_progress";
    case FailureCause::LocalizationInvalid:
      return "localization_invalid";
    case FailureCause::SensorInvalid:
      return "sensor_invalid";
    case FailureCause::ComputeDeadlineMiss:
      return "compute_deadline_miss";
    case FailureCause::LinkLost:
      return "link_lost";
  }
  return "none";
}

std::string to_string(EventKind kind) {
  switch (kind) {
    case EventKind::PlanAccepted:
      return "plan_accepted";
    case EventKind::PlanRejected:
      return "plan_rejected";
    case EventKind::WaypointReached:
      return "waypoint_reached";
    case EventKind::PlanComplete:
      return "plan_complete";
    case EventKind::PlanExpired:
      return "plan_expired";
    case EventKind::Blocked:
      return "blocked";
    case EventKind::FailsafeEntered:
      return "failsafe_entered";
    case EventKind::FailsafeDone:
      return "failsafe_done";
    case EventKind::FailsafeCleared:
      return "failsafe_cleared";
    case EventKind::Paused:
      return "paused";
  }
  return "blocked";
}

}  // namespace argus::node
