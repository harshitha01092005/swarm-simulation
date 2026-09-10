#ifndef DRONE_SWARM_FLEET_COLORS_HH_
#define DRONE_SWARM_FLEET_COLORS_HH_

#include "FleetProtocol.hh"

#include <cstdint>
#include <gz/msgs/visual.pb.h>

namespace drone_swarm
{
struct RgbColor
{
  double red{0};
  double green{0};
  double blue{0};

  bool operator==(const RgbColor &_other) const
  {
    return this->red == _other.red && this->green == _other.green &&
           this->blue == _other.blue;
  }
};

using FleetColors = std::vector<std::optional<RgbColor>>;

// Application-specific TFMessage/Pose_V transport, never a /tf transform:
// frame_id=world, child_frame_id=drone_NNN, position=RGB in [0,1], rotation=I.
// Parsing is atomic. Active membership is checked on the simulation thread
// immediately before applying a batch, after any pending pose reference.
inline std::optional<FleetColors> ParseColorBatch(
    const gz::msgs::Pose_V &_message, std::size_t _capacity)
{
  if (_capacity == 0 || _capacity > kMaxDrones ||
      static_cast<std::size_t>(_message.pose_size()) > _capacity)
    return std::nullopt;
  FleetColors colors(_capacity);
  for (const auto &pose : _message.pose())
  {
    const auto child = HeaderValue(pose.header(), "child_frame_id");
    const auto frame = HeaderValue(pose.header(), "frame_id");
    if (!child || !frame || *frame != "world" ||
        !pose.has_position() || !pose.has_orientation())
      return std::nullopt;
    const auto index = DroneIndex(*child, _capacity);
    if (!index || colors[*index] || (!pose.name().empty() && pose.name() != *child))
      return std::nullopt;
    const auto &rgb = pose.position();
    for (const double channel : {rgb.x(), rgb.y(), rgb.z()})
    {
      if (!std::isfinite(channel) || channel < 0.0 || channel > 1.0)
        return std::nullopt;
    }
    const auto &rotation = pose.orientation();
    if (rotation.w() != 1.0 || rotation.x() != 0.0 ||
        rotation.y() != 0.0 || rotation.z() != 0.0)
      return std::nullopt;
    colors[*index] = RgbColor{rgb.x(), rgb.y(), rgb.z()};
  }
  return colors;
}

inline bool ColorsMatchActive(const FleetColors &_colors,
                             const std::vector<bool> &_active)
{
  if (_colors.size() != _active.size())
    return false;
  for (std::size_t index = 0; index < _active.size(); ++index)
  {
    if (_colors[index].has_value() != _active[index])
      return false;
  }
  return true;
}

inline void SetMessageColor(gz::msgs::Color *_message, const RgbColor &_rgb)
{
  _message->set_r(static_cast<float>(_rgb.red));
  _message->set_g(static_cast<float>(_rgb.green));
  _message->set_b(static_cast<float>(_rgb.blue));
  _message->set_a(1.0f);
}

// Entity ID is resolved from owned model/base_link/show_light components,
// never accepted from a remote message. Emissive visuals are inexpensive;
// no extra shadow-casting point lights are allocated per drone.
inline gz::msgs::Visual ColorVisualCommand(std::uint64_t _entity,
                                          const RgbColor &_rgb)
{
  gz::msgs::Visual visual;
  visual.set_id(_entity);
  auto *material = visual.mutable_material();
  SetMessageColor(material->mutable_ambient(), _rgb);
  SetMessageColor(material->mutable_diffuse(), _rgb);
  SetMessageColor(material->mutable_emissive(), _rgb);
  SetMessageColor(material->mutable_specular(), RgbColor{});
  return visual;
}
}  // namespace drone_swarm

#endif  // DRONE_SWARM_FLEET_COLORS_HH_
