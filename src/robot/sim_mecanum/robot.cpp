#include "robot.hpp"

#include <thread.hpp>

#include "mod_chassis.hpp"
#include "system.hpp"

/* clang-format off */
Robot::Simulator::Param param = {
  .led = {
    .gpio = BSP_GPIO_LED,
    .timeout = 200,
  },

  .imu = {
    .tp_name_prefix = "chassis",
  },

  .chassis={
      .type = Component::Mixer::MECANUM,

      .follow_pid_param = {
      .k = 0.5f,
      .p = 1.0f,
      .i = 0.0f,
      .d = 0.0f,
      .i_limit = 1.0f,
      .out_limit = 1.0f,
      .d_cutoff_freq = -1.0f,
      .range = M_2PI,
    },

    .EVENT_MAP = {
      Component::CMD::EventMapItem{
        Device::TerminalController::STOP_CTRL,
        Module::RMChassis::SET_MODE_RELAX
      },
      Component::CMD::EventMapItem{
        Device::TerminalController::START_CTRL,
        Module::RMChassis::SET_MODE_INDENPENDENT
      },
    },

    .actuator_param = {
      Component::SpeedActuator::Param{
        .speed = {
          .k = 0.00025f,
          .p = 1.0f,
          .i = 0.0f,
          .d = 0.0f,
          .i_limit = 0.02f,
          .out_limit = 1.0f,
          .d_cutoff_freq = -1.0f,
          .range = -1.0f,
        },

        .in_cutoff_freq = -1.0f,

        .out_cutoff_freq = -1.0f,

      },
      Component::SpeedActuator::Param{
        .speed = {
          .k = 0.00025f,
          .p = 1.0f,
          .i = 0.0f,
          .d = 0.0f,
          .i_limit = 0.02f,
          .out_limit = 1.0f,
          .d_cutoff_freq = -1.0f,
          .range = -1.0f,
        },

        .in_cutoff_freq = -1.0f,

        .out_cutoff_freq = -1.0f,
      },
      Component::SpeedActuator::Param{
        .speed = {
          .k = 0.00025f,
          .p = 1.0f,
          .i = 0.0f,
          .d = 0.0f,
          .i_limit = 0.02f,
          .out_limit = 1.0f,
          .d_cutoff_freq = -1.0f,
          .range = -1.0f,
        },

        .in_cutoff_freq = -1.0f,

        .out_cutoff_freq = -1.0f,
      },
      Component::SpeedActuator::Param{
        .speed = {
          .k = 0.00025f,
          .p = 1.0f,
          .i = 0.0f,
          .d = 0.0f,
          .i_limit = 0.02f,
          .out_limit = 1.0f,
          .d_cutoff_freq = -1.0f,
          .range = -1.0f,
        },

        .in_cutoff_freq = -1.0f,

        .out_cutoff_freq = -1.0f,
      },
    },

    .motor_param = {
      Device::RMMotor::Param{
          .model = Device::RMMotor::MOTOR_M3508,
      },
      Device::RMMotor::Param{
          .model = Device::RMMotor::MOTOR_M3508,
      },
      Device::RMMotor::Param{
          .model = Device::RMMotor::MOTOR_M3508,
      },
      Device::RMMotor::Param{
          .model = Device::RMMotor::MOTOR_M3508,
      },
    },
  },
};
/* clang-format on */

void robot_init() {
  auto init_thread_fn = [](void* arg) {
    static_cast<void>(arg);

    System::Init();

    Robot::Simulator blink(param);

    while (1) {
      System::Thread::Sleep(UINT32_MAX);
    }
  };

  System::Thread init_thread;

  init_thread.Create(init_thread_fn, static_cast<void*>(NULL), "init_thread_fn",
                     512, System::Thread::REALTIME);
}
