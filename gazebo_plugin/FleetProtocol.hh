#ifndef DRONE_SWARM_FLEET_PROTOCOL_HH_
#define DRONE_SWARM_FLEET_PROTOCOL_HH_

#include <cmath>
#include <optional>
#include <string>
#include <vector>

#include <gz/math/Pose3.hh>
#include <gz/msgs/pose_v.pb.h>

namespace drone_swarm
{
constexpr std::size_t kMaxDrones = 400;
constexpr double kWorldLimit = 500.0;
constexpr double kMinimumAltitude = 0.5;

// Strict, unscoped model names prevent commands from addressing scene assets.
inline std::optional<std::size_t> DroneIndex(const std::string &_name,
                                          std::size_t _capacity)
{
  if (_name.size() != 9 || _name.compare(0, 6, "drone_") != 0)
    return std::nullopt;
  std::size_t number = 0;
  for (std::size_t i = 6; i < 9; ++i)
  {
    if (_name[i] < '0' || _name[i] > '9')
      return std::nullopt;
    number = number * 10 + static_cast<std::size_t>(_name[i] - '0');
  }
  if (number == 0 || number > _capacity || number > kMaxDrones)
    return std::nullopt;
  return number - 1;
}

inline std::optional<std::string> HeaderValue(const gz::msgs::Header &_header,
                                            const std::string &_key)
{
  std::optional<std::string> result;
  for (const auto &entry : _header.data())
  {
    if (entry.key() == _key)
    {
      if (result || entry.value_size() != 1)
        return std::nullopt;
      result = entry.value(0);
    }
  }
  return result;
}

using FleetBatch = std::vector<std::optional<gz::math::Pose3d>>;

// A batch is atomic and specifies the complete active fleet. Empty batches
// deactivate every drone. Never partially apply a malformed formation.
inline std::optional<FleetBatch> ParseBatch(const gz::msgs::Pose_V &_message,
                                         std::size_t _capacity)
{
  if (_capacity == 0 || _capacity > kMaxDrones ||
      static_cast<std::size_t>(_message.pose_size()) > _capacity)
    return std::nullopt;
  FleetBatch batch(_capacity);
  for (const auto &pose : _message.pose())
  {
    // Jazzy TransformStamped conversion stores identities in Header data;
    // Pose.name is not populated by the ROS->Gazebo bridge.
    const auto child = HeaderValue(pose.header(), "child_frame_id");
    const auto frame = HeaderValue(pose.header(), "frame_id");
    if (!child || !frame || *frame != "world")
      return std::nullopt;
    const auto index = DroneIndex(*child, _capacity);
    if (!index || batch[*index] || (!pose.name().empty() && pose.name() != *child))
      return std::nullopt;
    const auto &p = pose.position();
    const auto &q = pose.orientation();
    if (!std::isfinite(p.x()) || !std::isfinite(p.y()) || !std::isfinite(p.z()) ||
        std::abs(p.x()) > kWorldLimit || std::abs(p.y()) > kWorldLimit ||
        p.z() < kMinimumAltitude || p.z() > kWorldLimit ||
        !std::isfinite(q.w()) || !std::isfinite(q.x()) ||
        !std::isfinite(q.y()) || !std::isfinite(q.z()))
      return std::nullopt;
    const double norm = std::hypot(std::hypot(q.w(), q.x()), std::hypot(q.y(), q.z()));
    if (!std::isfinite(norm) || norm < 1e-6 || norm > 1e6)
      return std::nullopt;
    batch[*index] = gz::math::Pose3d(
        gz::math::Vector3d(p.x(), p.y(), p.z()),
        gz::math::Quaterniond(q.w() / norm, q.x() / norm, q.y() / norm, q.z() / norm));
  }
  return batch;
}
}  // namespace drone_swarm

#endif  // DRONE_SWARM_FLEET_PROTOCOL_HH_
