#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include <gz/math/Vector3.hh>
#include <gz/msgs/double.pb.h>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/transport/Node.hh>
#include <sdf/Element.hh>

namespace kty_conveyor_surface
{
struct Zone
{
  std::string name;
  std::string topic;
  double minX{0.0};
  double maxX{0.0};
  double minY{-0.35};
  double maxY{0.35};
  double command{0.0};
};

class KtyConveyorSurfaceSystem final:
  public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate
{
public:
  void Configure(
    const gz::sim::Entity &,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &,
    gz::sim::EventManager &) override
  {
    if (_sdf->HasElement("model_prefix"))
      this->modelPrefix = _sdf->Get<std::string>("model_prefix");
    if (_sdf->HasElement("surface_z"))
      this->surfaceZ = _sdf->Get<double>("surface_z");
    if (_sdf->HasElement("contact_tolerance"))
      this->contactTolerance = _sdf->Get<double>("contact_tolerance");
    if (_sdf->HasElement("velocity_gain"))
      this->velocityGain = _sdf->Get<double>("velocity_gain");
    if (_sdf->HasElement("max_force"))
      this->maxForce = _sdf->Get<double>("max_force");
    if (_sdf->HasElement("velocity_deadband"))
      this->velocityDeadband = _sdf->Get<double>("velocity_deadband");
    if (_sdf->HasElement("pose_registry_topic"))
      this->poseRegistryTopic = _sdf->Get<std::string>("pose_registry_topic");
    if (_sdf->HasElement("pose_registry_prefix"))
      this->poseRegistryPrefix = _sdf->Get<std::string>("pose_registry_prefix");
    if (_sdf->HasElement("pose_registry_hz"))
      this->poseRegistryHz = _sdf->Get<double>("pose_registry_hz");

    this->poseRegistryHz = std::max(1.0, this->poseRegistryHz);
    this->registryPublisher =
      this->transport.Advertise<gz::msgs::StringMsg>(this->poseRegistryTopic);

    if (!_sdf->HasElement("zone"))
      return;

    // sdformat14 exposes repeated-child traversal through a non-const API.
    // The tree is only traversed and is never modified here.
    auto *mutableSdf = const_cast<sdf::Element *>(_sdf.get());
    auto zoneElement = mutableSdf->GetElement("zone");
    while (zoneElement)
    {
      Zone zone;
      zone.name = zoneElement->Get<std::string>("name");
      zone.topic = zoneElement->Get<std::string>("topic");
      zone.minX = zoneElement->Get<double>("min_x");
      zone.maxX = zoneElement->Get<double>("max_x");
      if (zoneElement->HasElement("min_y"))
        zone.minY = zoneElement->Get<double>("min_y");
      if (zoneElement->HasElement("max_y"))
        zone.maxY = zoneElement->Get<double>("max_y");
      this->zones.push_back(std::move(zone));
      zoneElement = zoneElement->GetNextElement("zone");
    }

    for (std::size_t index = 0; index < this->zones.size(); ++index)
    {
      const auto topic = this->zones[index].topic;
      this->transport.Subscribe<gz::msgs::Double>(
        topic,
        [this, index](const gz::msgs::Double &_message)
        {
          std::lock_guard<std::mutex> guard(this->commandMutex);
          this->zones[index].command = _message.data();
        });
    }
  }

  void PreUpdate(
    const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm) override
  {
    if (_info.paused)
      return;

    std::vector<double> commands;
    {
      std::lock_guard<std::mutex> guard(this->commandMutex);
      commands.reserve(this->zones.size());
      for (const auto &zone : this->zones)
        commands.push_back(zone.command);
    }

    const double simSeconds =
      std::chrono::duration<double>(_info.simTime).count();
    const bool publishRegistry =
      simSeconds - this->lastRegistryPublishSeconds >=
      1.0 / this->poseRegistryHz;

    std::ostringstream registry;
    bool firstRegistryModel = true;
    if (publishRegistry)
    {
      registry << std::fixed << std::setprecision(9)
               << "{\"schema\":\"kty_model_pose_registry/v1\","
               << "\"sequence\":" << ++this->registrySequence << ','
               << "\"sim_time_s\":" << simSeconds << ','
               << "\"models\":[";
    }

    _ecm.Each<gz::sim::components::Model, gz::sim::components::Name>(
      [this, &commands, &_ecm, publishRegistry, &registry,
       &firstRegistryModel](
        const gz::sim::Entity &_entity,
        const gz::sim::components::Model *,
        const gz::sim::components::Name *_name) -> bool
      {
        const auto &name = _name->Data();
        const bool isTransportModel = name.rfind(this->modelPrefix, 0) == 0;
        const bool isRegistryModel =
          name.rfind(this->poseRegistryPrefix, 0) == 0;
        if (!isTransportModel && !(publishRegistry && isRegistryModel))
          return true;

        gz::sim::Model model(_entity);
        const auto linkEntity = model.CanonicalLink(_ecm);
        if (linkEntity == gz::sim::kNullEntity)
          return true;

        gz::sim::Link link(linkEntity);
        const auto pose = link.WorldPose(_ecm);
        if (!pose)
          return true;

        if (publishRegistry && isRegistryModel)
        {
          if (!firstRegistryModel)
            registry << ',';
          firstRegistryModel = false;
          registry << "{\"name\":\"" << EscapeJson(name) << "\","
                   << "\"x\":" << pose->Pos().X() << ','
                   << "\"y\":" << pose->Pos().Y() << ','
                   << "\"z\":" << pose->Pos().Z() << '}';
        }

        if (!isTransportModel || this->zones.empty())
          return true;

        if (this->velocityEnabled.insert(linkEntity).second)
          link.EnableVelocityChecks(_ecm, true);
        const auto velocity = link.WorldLinearVelocity(_ecm);
        if (!velocity)
          return true;

        const auto &position = pose->Pos();
        if (std::abs(position.Z() - this->surfaceZ) > this->contactTolerance)
          return true;

        bool insideZone = false;
        double targetVelocity = 0.0;
        // Later zones have priority in overlap regions, so an exiting KTY
        // cannot remain bound to a zero-speed active zone at the hand-off.
        for (std::size_t index = 0; index < this->zones.size(); ++index)
        {
          const auto &zone = this->zones[index];
          if (position.X() >= zone.minX && position.X() <= zone.maxX &&
              position.Y() >= zone.minY && position.Y() <= zone.maxY)
          {
            insideZone = true;
            targetVelocity = commands[index];
          }
        }
        if (!insideZone)
          return true;

        if (std::abs(targetVelocity) > this->velocityDeadband)
        {
          // The flat conveyor is an abstract velocity-imposing contact surface.
          // Apply only horizontal transport; preserve current Y/Z velocity so
          // gravity and the loaded products remain physical. Suppressing angular
          // velocity while driven prevents a lower edge contact from overturning
          // the complete KTY. When the command is zero, no velocity command is
          // issued and normal vibration / collision physics is untouched.
          link.SetLinearVelocity(
            _ecm,
            gz::math::Vector3d(targetVelocity, velocity->Y(), velocity->Z()));
          link.SetAngularVelocity(_ecm, gz::math::Vector3d::Zero);
          return true;
        }

        // With the conveyor stopped, use only a mild longitudinal brake. This
        // avoids freezing vertical dynamics during loading and compaction.
        const double error = -velocity->X();
        double force = std::clamp(
          this->velocityGain * error,
          -this->maxForce,
          this->maxForce);
        if (std::abs(velocity->X()) <= this->velocityDeadband)
          force = 0.0;
        link.AddWorldForce(_ecm, gz::math::Vector3d(force, 0.0, 0.0));
        return true;
      });

    if (publishRegistry)
    {
      registry << "]}";
      gz::msgs::StringMsg message;
      message.set_data(registry.str());
      this->registryPublisher.Publish(message);
      this->lastRegistryPublishSeconds = simSeconds;
    }
  }

private:
  static std::string EscapeJson(const std::string &_value)
  {
    std::string result;
    result.reserve(_value.size());
    for (const char character : _value)
    {
      switch (character)
      {
        case '\\': result += "\\\\"; break;
        case '"': result += "\\\""; break;
        case '\n': result += "\\n"; break;
        case '\r': result += "\\r"; break;
        case '\t': result += "\\t"; break;
        default: result += character; break;
      }
    }
    return result;
  }

  std::string modelPrefix{"kty_mech_container_"};
  double surfaceZ{0.50};
  double contactTolerance{0.075};
  double velocityGain{80.0};
  double maxForce{120.0};
  double velocityDeadband{0.005};
  std::string poseRegistryTopic{"/kty/mech/model_pose_registry_json"};
  std::string poseRegistryPrefix{"kty_mech_"};
  double poseRegistryHz{20.0};
  double lastRegistryPublishSeconds{-1.0e9};
  std::uint64_t registrySequence{0};
  std::vector<Zone> zones;
  std::mutex commandMutex;
  std::unordered_set<gz::sim::Entity> velocityEnabled;
  gz::transport::Node transport;
  gz::transport::Node::Publisher registryPublisher;
};
}

GZ_ADD_PLUGIN(
  kty_conveyor_surface::KtyConveyorSurfaceSystem,
  gz::sim::System,
  gz::sim::ISystemConfigure,
  gz::sim::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(
  kty_conveyor_surface::KtyConveyorSurfaceSystem,
  "kty_conveyor_surface::KtyConveyorSurfaceSystem")