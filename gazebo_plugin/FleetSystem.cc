// Batched kinematic visualization backend for Gazebo Harmonic (gz-sim8).
// Static bodies follow sampled references; there is no motor or lift model.
#include "FleetProtocol.hh"
#include "FleetColors.hh"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <utility>

#include <gz/common/Console.hh>
#include <gz/math/Color.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Material.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/Static.hh>
#include <gz/sim/components/Visual.hh>
#include <gz/sim/components/VisualCmd.hh>
#include <gz/sim/components/World.hh>
#include <gz/transport/Node.hh>

namespace drone_swarm
{
namespace components = gz::sim::components;

class FleetSystem final : public gz::sim::System,
                          public gz::sim::ISystemConfigure,
                          public gz::sim::ISystemPreUpdate,
                          public gz::sim::ISystemPostUpdate,
                          public gz::sim::ISystemReset
{
 public:
  ~FleetSystem() override
  {
    // Stop callbacks before their mutex / command storage are destroyed.
    this->node.Unsubscribe(this->referenceTopic);
    this->node.Unsubscribe(this->colorTopic);
  }

  void Configure(const gz::sim::Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 gz::sim::EntityComponentManager &_ecm,
                 gz::sim::EventManager &) override
  {
    if (!_ecm.Component<components::World>(_entity))
      throw std::runtime_error("FleetSystem must be attached to a world");
    this->world = _entity;
    this->capacity = _sdf->Get<unsigned int>("model_count", 5).first;
    const auto frequency = _sdf->Get<double>("publish_frequency", 30.0).first;
    if (this->capacity == 0 || this->capacity > kMaxDrones ||
        !std::isfinite(frequency) || frequency < 1.0 || frequency > 120.0)
      throw std::runtime_error("FleetSystem requires 1..400 models and 1..120 Hz");
    this->period = 1.0 / frequency;
    const auto colorFrequency = _sdf->Get<double>("color_update_frequency", 10.0).first;
    if (!std::isfinite(colorFrequency) || colorFrequency < 1.0 || colorFrequency > 30.0)
      throw std::runtime_error("FleetSystem color updates require 1..30 Hz");
    this->colorPeriod = 1.0 / colorFrequency;
    this->referenceTopic = _sdf->Get<std::string>(
        "reference_topic", "/swarm/reference").first;
    const auto poseTopic = _sdf->Get<std::string>("pose_topic", "/swarm/poses").first;
    this->colorTopic = _sdf->Get<std::string>("color_topic", "/swarm/colors").first;
    this->entities.assign(this->capacity, gz::sim::kNullEntity);
    this->ledEntities.assign(this->capacity, gz::sim::kNullEntity);
    this->appliedColors.resize(this->capacity);
    this->active.assign(this->capacity, true);
    for (std::size_t i = 0; i < this->capacity; ++i)
    {
      std::ostringstream name;
      name << "drone_" << std::setfill('0') << std::setw(3) << i + 1;
      this->names.push_back(name.str());
    }
    this->publisher = this->node.Advertise<gz::msgs::Pose_V>(poseTopic);
    if (!this->publisher || !this->node.Subscribe(
        this->referenceTopic, &FleetSystem::OnReference, this) ||
        !this->node.Subscribe(this->colorTopic, &FleetSystem::OnColors, this))
      throw std::runtime_error("FleetSystem failed to connect Gazebo transport topics");
    gzmsg << "FleetSystem: " << this->capacity
          << " static kinematic models; references " << this->referenceTopic
          << ", measured ECM poses " << poseTopic << " at " << frequency << " Hz.\n";
    gzmsg << "FleetSystem: RGB batches " << this->colorTopic
          << ", emissive show_light updates at most " << colorFrequency << " Hz.\n";
  }

  void PreUpdate(const gz::sim::UpdateInfo &_info,
                 gz::sim::EntityComponentManager &_ecm) override
  {
    if (!this->cached)
      this->CacheModels(_ecm);
    if (_info.paused)
      return;

    std::optional<FleetBatch> command;
    {
      std::lock_guard<std::mutex> lock(this->commandMutex);
      command.swap(this->latestCommand);
    }
    if (!command)
    {
      this->ApplyColors(_info, _ecm);
      return;
    }

    for (std::size_t i = 0; i < this->capacity; ++i)
    {
      const bool wasActive = this->active[i];
      this->active[i] = (*command)[i].has_value();
      if (!this->active[i] && !wasActive)
        continue;
      const auto entity = this->entities[i];
      if (!_ecm.Component<components::Model>(entity))
        continue;
      // These cached static models are direct children of the world, so their
      // Pose component is also their world pose. Update owned ECS state without
      // a physics traversal; PeriodicChange propagates motion to scene clients.
      const auto pose = (*command)[i].value_or(
          gz::math::Pose3d(0, 0, -100.0 - static_cast<double>(i), 0, 0, 0));
      auto *component = _ecm.Component<components::Pose>(entity);
      if (component && component->Data() != pose)
      {
        component->Data() = pose;
        _ecm.SetChanged(entity, components::Pose::typeId,
                       gz::sim::ComponentState::PeriodicChange);
      }
    }
    this->ApplyColors(_info, _ecm);
  }

  void PostUpdate(const gz::sim::UpdateInfo &_info,
                  const gz::sim::EntityComponentManager &_ecm) override
  {
    if (!this->cached)
      return;
    const double now = std::chrono::duration<double>(_info.simTime).count();
    if (now < this->lastTime)
      this->nextPublish = now;
    this->lastTime = now;
    if (now + 1e-9 < this->nextPublish)
      return;
    // Advance from the intended deadline, avoiding accumulated rounding drift.
    this->nextPublish += this->period;
    if (this->nextPublish <= now)
      this->nextPublish = now + this->period;

    gz::msgs::Pose_V message;
    this->SetHeader(message.mutable_header(), _info);
    for (std::size_t i = 0; i < this->capacity; ++i)
    {
      if (!this->active[i] || !_ecm.Component<components::Pose>(this->entities[i]))
        continue;
      const auto actual = gz::sim::worldPose(this->entities[i], _ecm);
      auto *pose = message.add_pose();
      pose->set_name(this->names[i]);
      pose->set_id(this->entities[i]);
      this->SetHeader(pose->mutable_header(), _info);
      auto *child = pose->mutable_header()->add_data();
      child->set_key("child_frame_id");
      child->add_value(this->names[i]);
      pose->mutable_position()->set_x(actual.Pos().X());
      pose->mutable_position()->set_y(actual.Pos().Y());
      pose->mutable_position()->set_z(actual.Pos().Z());
      pose->mutable_orientation()->set_w(actual.Rot().W());
      pose->mutable_orientation()->set_x(actual.Rot().X());
      pose->mutable_orientation()->set_y(actual.Rot().Y());
      pose->mutable_orientation()->set_z(actual.Rot().Z());
    }
    this->publisher.Publish(message);
  }

  void Reset(const gz::sim::UpdateInfo &,
             gz::sim::EntityComponentManager &) override
  {
    std::lock_guard<std::mutex> lock(this->commandMutex);
    this->latestCommand.reset();
    std::fill(this->active.begin(), this->active.end(), true);
    this->nextPublish = 0;
    this->lastTime = 0;
    {
      std::lock_guard<std::mutex> colorLock(this->colorMutex);
      this->latestColors.reset();
    }
    std::fill(this->appliedColors.begin(), this->appliedColors.end(), std::nullopt);
    this->nextColorUpdate = 0;
    this->lastColorTime = 0;
  }

 private:
  void CacheModels(gz::sim::EntityComponentManager &_ecm)
  {
    // Exactly one world scan. Runtime work uses these stable entity IDs and
    // never discovers or modifies later arbitrary scene additions.
    _ecm.Each<components::Model, components::Name,
              components::ParentEntity, components::Static>(
        [&](const gz::sim::Entity &_entity, const components::Model *,
            const components::Name *_name, const components::ParentEntity *_parent,
            const components::Static *_static)
        {
          const auto index = DroneIndex(_name->Data(), this->capacity);
          if (index && _parent->Data() == this->world && _static->Data())
          {
            if (this->entities[*index] != gz::sim::kNullEntity)
              throw std::runtime_error("FleetSystem: duplicate owned model name");
            this->entities[*index] = _entity;
          }
          return true;
        });
    if (std::find(this->entities.begin(), this->entities.end(), gz::sim::kNullEntity)
        != this->entities.end())
      throw std::runtime_error("FleetSystem: expected static drone models are missing");
    for (std::size_t i = 0; i < this->capacity; ++i)
    {
      const auto links = _ecm.ChildrenByComponents(this->entities[i],
          components::Link(), components::Name("base_link"));
      if (links.size() != 1)
        throw std::runtime_error("FleetSystem: owned drone base_link is missing or ambiguous");
      const auto visuals = _ecm.ChildrenByComponents(links[0],
          components::Visual(), components::Name("show_light"));
      if (visuals.size() != 1 ||
          !_ecm.Component<components::Material>(visuals[0]))
        throw std::runtime_error("FleetSystem: owned show_light material is missing or ambiguous");
      this->ledEntities[i] = visuals[0];
    }
    this->cached = true;
  }

  void OnColors(const gz::msgs::Pose_V &_message)
  {
    auto colors = ParseColorBatch(_message, this->capacity);
    if (!colors)
    {
      this->RejectColors("malformed RGB values or identities");
      return;
    }
    std::lock_guard<std::mutex> lock(this->colorMutex);
    // Coalesce a 30 Hz publisher to the latest batch; never accumulate a queue.
    this->latestColors = std::move(colors);
  }

  void RejectColors(const char *_reason)
  {
    const auto rejected = ++this->rejectedColorBatches;
    if (rejected == 1 || rejected % 100 == 0)
      gzwarn << "FleetSystem rejected RGB batch: " << _reason
             << " (" << rejected << " rejected).\n";
  }

  void ApplyColors(const gz::sim::UpdateInfo &_info,
                   gz::sim::EntityComponentManager &_ecm)
  {
    const double now = std::chrono::duration<double>(_info.simTime).count();
    if (now < this->lastColorTime)
      this->nextColorUpdate = now;
    this->lastColorTime = now;
    if (now + 1e-9 < this->nextColorUpdate)
      return;
    std::optional<FleetColors> colors;
    {
      std::lock_guard<std::mutex> lock(this->colorMutex);
      colors.swap(this->latestColors);
    }
    if (!colors)
      return;
    this->nextColorUpdate = now + this->colorPeriod;
    // active is accessed only on the simulation thread, after pose membership
    // changes. Independent transport callback ordering cannot create a race.
    if (!ColorsMatchActive(*colors, this->active))
    {
      this->RejectColors("colors must contain exactly the active model IDs");
      return;
    }
    for (std::size_t i = 0; i < this->capacity; ++i)
    {
      if (this->active[i] &&
          (!_ecm.Component<components::Visual>(this->ledEntities[i]) ||
           !_ecm.Component<components::Material>(this->ledEntities[i])))
      {
        this->RejectColors("an owned show_light visual was removed");
        return;
      }
    }
    for (std::size_t i = 0; i < this->capacity; ++i)
    {
      if (!this->active[i] || (this->appliedColors[i] &&
          *this->appliedColors[i] == *(*colors)[i]))
        continue;
      const auto entity = this->ledEntities[i];
      const auto &rgb = *(*colors)[i];
      const gz::math::Color color(static_cast<float>(rgb.red),
          static_cast<float>(rgb.green), static_cast<float>(rgb.blue), 1.0f);
      auto *material = _ecm.Component<components::Material>(entity);
      auto updated = material->Data();
      updated.SetAmbient(color);
      updated.SetDiffuse(color);
      updated.SetEmissive(color);
      updated.SetSpecular(gz::math::Color(0.0f, 0.0f, 0.0f, 1.0f));
      // Keep persistent material state correct even without a rendering client.
      material->Data() = updated;
      _ecm.SetChanged(entity, components::Material::typeId,
                      gz::sim::ComponentState::OneTimeChange);

      // Same supported command path as gz-sim8 UserCommands::VisualCommand.
      // RenderUtil consumes VisualCmd to update live rendered emissive colors.
      const auto visual = ColorVisualCommand(entity, rgb);
      auto *visualCmd = _ecm.Component<components::VisualCmd>(entity);
      if (visualCmd)
      {
        visualCmd->SetData(visual, [](const auto &, const auto &) { return false; });
        _ecm.SetChanged(entity, components::VisualCmd::typeId,
                        gz::sim::ComponentState::OneTimeChange);
      }
      else
      {
        _ecm.CreateComponent(entity, components::VisualCmd(visual));
      }
      this->appliedColors[i] = rgb;
    }
  }

  void OnReference(const gz::msgs::Pose_V &_message)
  {
    auto parsed = ParseBatch(_message, this->capacity);
    if (!parsed)
    {
      const auto rejected = ++this->rejectedBatches;
      if (rejected == 1 || rejected % 100 == 0)
        gzwarn << "FleetSystem rejected malformed or out-of-bounds batch ("
               << rejected << " rejected).\n";
      return;
    }
    std::lock_guard<std::mutex> lock(this->commandMutex);
    this->latestCommand = std::move(parsed);
  }

  static void SetHeader(gz::msgs::Header *_header,
                        const gz::sim::UpdateInfo &_info)
  {
    const auto nanos = std::chrono::duration_cast<std::chrono::nanoseconds>(
        _info.simTime).count();
    _header->mutable_stamp()->set_sec(nanos / 1000000000);
    _header->mutable_stamp()->set_nsec(static_cast<int32_t>(nanos % 1000000000));
    auto *frame = _header->add_data();
    frame->set_key("frame_id");
    frame->add_value("world");
  }

  gz::sim::Entity world{gz::sim::kNullEntity};
  std::size_t capacity{0};
  double period{1.0 / 30.0};
  double nextPublish{0};
  double lastTime{0};
  double colorPeriod{0.1};
  double nextColorUpdate{0};
  double lastColorTime{0};
  bool cached{false};
  std::vector<gz::sim::Entity> entities;
  std::vector<gz::sim::Entity> ledEntities;
  std::vector<std::string> names;
  std::vector<bool> active;
  std::mutex commandMutex;
  std::optional<FleetBatch> latestCommand;
  std::mutex colorMutex;
  std::optional<FleetColors> latestColors;
  FleetColors appliedColors;
  std::atomic<std::uint64_t> rejectedBatches{0};
  std::atomic<std::uint64_t> rejectedColorBatches{0};
  std::string referenceTopic;
  std::string colorTopic;
  gz::transport::Node node;
  gz::transport::Node::Publisher publisher;
};
}  // namespace drone_swarm

GZ_ADD_PLUGIN(drone_swarm::FleetSystem, gz::sim::System,
              gz::sim::ISystemConfigure, gz::sim::ISystemPreUpdate,
              gz::sim::ISystemPostUpdate, gz::sim::ISystemReset)
GZ_ADD_PLUGIN_ALIAS(drone_swarm::FleetSystem, "drone_swarm::FleetSystem")
