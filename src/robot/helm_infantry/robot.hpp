/* #include "dev_xxx.hpp" */
#include "comp_cmd.hpp"
#include "dev_ahrs.hpp"
#include "dev_aim.hpp"
#include "dev_bmi088.hpp"
#include "dev_can.hpp"
#include "dev_cap.hpp"
#include "dev_dr16.hpp"
#include "dev_led_rgb.hpp"
#include "dev_referee.hpp"
#include "mod_gimbal.hpp"
#include "mod_helm_chassis.hpp"
#include "mod_launcher.hpp"

void robot_init();

namespace Robot {
class HelmInfantry {
 public:
  typedef struct Param {
    Module::RMHelmChassis::Param chassis;
    Module::Gimbal::Param gimbal;
    Module::RMLauncher::Param launcher;
    Device::BMI088::Rotation bmi088_rot{};
    Device::Cap::Param cap{};
  } Param;

  Component::CMD cmd_;

  Device::Referee referee_;
  Device::Can can_;
  Device::AIM aim_;
  Device::AHRS ahrs_;
  Device::BMI088 bmi088_;
  Device::Cap cap_;
  Device::DR16 dr16_;
  Device::RGB led_;

  Module::RMHelmChassis chassis_;
  Module::Gimbal gimbal_;
  Module::RMLauncher launcher_;

  HelmInfantry(Param& param, float control_freq)
      : bmi088_(param.bmi088_rot),
        cap_(param.cap),
        chassis_(param.chassis, control_freq),
        gimbal_(param.gimbal, control_freq),
        launcher_(param.launcher, control_freq) {}
};
}  // namespace Robot
