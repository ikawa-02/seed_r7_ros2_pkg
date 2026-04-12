#pragma once

#include <memory>
#include <vector>
#include <string>

#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/math/PID.hh>
#include <sdf/Element.hh>

namespace mimicplugin
{
  struct MimicConfig {
    gz::sim::Entity src_entity{gz::sim::kNullEntity};
    gz::sim::Entity dst_entity{gz::sim::kNullEntity};
    double offset{0.0};
    double multiplier{1.0};
    gz::math::PID pid;
    bool velocity{false};
  };

  class MimicPlugin :
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate
  {
  public:
    void Configure(
      const gz::sim::Entity &_entity,
      const std::shared_ptr<const sdf::Element> &_sdf,
      gz::sim::EntityComponentManager &_ecm,
      gz::sim::EventManager &_eventMgr) override;

    void PreUpdate(
      const gz::sim::UpdateInfo &_info,
      gz::sim::EntityComponentManager &_ecm) override;

  private:
    gz::sim::Model model_{gz::sim::kNullEntity};
    std::vector<MimicConfig> mimics_;
  };
}
