#include "MimicPlugin.h"

#include <gz/plugin/Register.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointVelocityCmd.hh>
#include <gz/sim/components/JointForceCmd.hh>

#include <iostream>
#include <chrono>

GZ_ADD_PLUGIN(
  mimicplugin::MimicPlugin,
  gz::sim::System,
  gz::sim::ISystemConfigure,
  gz::sim::ISystemPreUpdate
)

GZ_ADD_PLUGIN_ALIAS(mimicplugin::MimicPlugin, "mimicplugin::MimicPlugin")

void mimicplugin::MimicPlugin::Configure(
  const gz::sim::Entity &_entity,
  const std::shared_ptr<const sdf::Element> &_sdf,
  gz::sim::EntityComponentManager &_ecm,
  gz::sim::EventManager &)
{
  this->model_ = gz::sim::Model(_entity);
  std::cerr << "MimicPlugin: Loading" << std::endl;

  // sdf::Element::GetElement is non-const, so cast away const
  auto sdf = std::const_pointer_cast<sdf::Element>(
    std::const_pointer_cast<sdf::Element>(_sdf));

  if (!sdf->HasElement("mimic")) return;

  auto el = sdf->GetElement("mimic");
  while (el) {
    std::string target_name = el->Get<std::string>();
    std::string source_name;
    double offset = 0.0;
    double multiplier = 1.0;

    if (auto attr = el->GetAttribute("joint")) {
      attr->Get(source_name);
    }

    if (source_name.empty()) {
      el = el->GetNextElement("mimic");
      continue;
    }

    if (auto attr = el->GetAttribute("offset"))     attr->Get(offset);
    if (auto attr = el->GetAttribute("multiplier")) attr->Get(multiplier);

    double p = 100, i = 0, d = 0.1;
    double imax = 0, imin = 0, cmax = 1000, cmin = -1000;
    bool vel = false;
    if (auto a = el->GetAttribute("P"))           a->Get(p);
    if (auto a = el->GetAttribute("I"))           a->Get(i);
    if (auto a = el->GetAttribute("D"))           a->Get(d);
    if (auto a = el->GetAttribute("i_max"))       a->Get(imax);
    if (auto a = el->GetAttribute("i_min"))       a->Get(imin);
    if (auto a = el->GetAttribute("command_max")) a->Get(cmax);
    if (auto a = el->GetAttribute("command_min")) a->Get(cmin);
    if (auto a = el->GetAttribute("velocity"))    a->Get(vel);

    auto src_entity = this->model_.JointByName(_ecm, source_name);
    auto dst_entity = this->model_.JointByName(_ecm, target_name);

    if (src_entity == gz::sim::kNullEntity || dst_entity == gz::sim::kNullEntity) {
      std::cerr << "MimicPlugin: joint not found: "
                << source_name << " or " << target_name << std::endl;
      el = el->GetNextElement("mimic");
      continue;
    }

    MimicConfig cfg;
    cfg.src_entity = src_entity;
    cfg.dst_entity = dst_entity;
    cfg.offset = offset;
    cfg.multiplier = multiplier;
    cfg.pid.Init(p, i, d, imax, imin, cmax, cmin);
    cfg.velocity = vel;

    // Enable position reading on both joints
    gz::sim::Joint(src_entity).EnablePositionCheck(_ecm, true);
    gz::sim::Joint(dst_entity).EnablePositionCheck(_ecm, true);

    this->mimics_.push_back(cfg);
    std::cerr << "MimicPlugin: registered " << source_name
              << " -> " << target_name
              << " (offset=" << offset << " mult=" << multiplier << ")" << std::endl;

    el = el->GetNextElement("mimic");
  }
}

void mimicplugin::MimicPlugin::PreUpdate(
  const gz::sim::UpdateInfo &_info,
  gz::sim::EntityComponentManager &_ecm)
{
  if (_info.paused) return;

  double dt = std::chrono::duration<double>(_info.dt).count();
  if (dt <= 0.0) return;

  for (auto &cfg : this->mimics_) {
    gz::sim::Joint src(cfg.src_entity);
    gz::sim::Joint dst(cfg.dst_entity);

    auto src_pos = src.Position(_ecm);
    auto dst_pos = dst.Position(_ecm);

    if (!src_pos || src_pos->empty()) continue;
    if (!dst_pos || dst_pos->empty()) continue;

    double s_current = (*src_pos)[0];
    double t_current = (*dst_pos)[0];
    double t_desired = (s_current - cfg.offset) * cfg.multiplier;

    double result = cfg.pid.Update(
      t_current - t_desired,
      std::chrono::duration<double>(dt));

    if (cfg.velocity) {
      dst.SetVelocity(_ecm, {result});
    } else {
      dst.SetForce(_ecm, {result});
    }
  }
}
