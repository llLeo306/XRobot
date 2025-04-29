#include "robot.hpp"

#include <comp_actuator.hpp>
#include <system.hpp>
#include <thread.hpp>

#include "dev_rm_motor.hpp"

/* clang-format off */
Robot::HelmInfantry::Param param =
    {

    //   .robot_st =
    //   {
    //     .k=0.038,//0.092
    //     .bullet_type=Device::NewAi::BULLET_17,
    //     .bias_time=60,
    //     .s_bias=0.21500,
    //     .z_bias=0.12500,
    //   },

        .chassis =
            {
                // .toque_coefficient_3508 = 0.044876589f,
                // .speed_2_coefficient_3508 = 1.563578147703433e-19f,
                // .out_2_coefficient_3508 = 54.67448580571495f,
                // .constant_3508 = 6.564590719535691f,
                .toque_coefficient_3508 = 0.04585928841670807f,
                .speed_2_coefficient_3508  = 1.7539151934403435e-18f,
                .out_2_coefficient_3508 = 26.953338626324133f,
                .constant_3508 = 4.220175002051428e-09f,


                .follow_pid_param =
                {
                        .k = 5.f,
                        .p = 1.3f,
                        .i = 0.8f,
                        .d = 0.05f,
                        .i_limit = 1.2f,
                        .out_limit = 6.0f,
                        .d_cutoff_freq = -1.0f,
                        .cycle = true,
                },

                .EVENT_MAP =
                    {Component::CMD::EventMapItem{
                         Component::CMD::CMD_EVENT_LOST_CTRL,
                         Module::RMHelmChassis::SET_MODE_RELAX},
                     Component::CMD::EventMapItem{
                         Device::DR16::DR16_SW_L_POS_TOP,
                         Module::RMHelmChassis::SET_MODE_RELAX},
                     Component::CMD::EventMapItem{
                         Device::DR16::DR16_SW_L_POS_MID,
                         Module::RMHelmChassis::SET_MODE_CHASSIS_FOLLOW},
                     Component::CMD::EventMapItem{
                         Device::DR16::DR16_SW_L_POS_BOT,
                         Module::RMHelmChassis::SET_MODE_ROTOR},
                     Component::CMD::EventMapItem{
                         Device::DR16::KEY_V,
                         Module::RMHelmChassis::SET_MODE_ROTOR},
                     Component::CMD::EventMapItem{
                         Device::DR16::KEY_B,
                         Module::RMHelmChassis::SET_MODE_CHASSIS_FOLLOW},
                     Component::CMD::EventMapItem{
                         Device::DR16::KEY_F,
                         Module::RMHelmChassis::SET_MODE_6020_FOLLOW}
                         },


                .pos_param =
                    {
                        Component::PosActuator::Param{
                            .speed =
                                {
                                    .k = 0.005f,
                                    .p = 1.0f,//1.0f,
                                    .i = 0.4f,//0.3f
                                    .d = 0.001f,
                                    .i_limit = 0.25f,
                                    .out_limit = 0.6f,
                                    .d_cutoff_freq = -1.0f,
                                    .cycle = false,
                                },
                            .position =
                                {
                                    .k = 150.0f,
                                    .p = 1.5f,
                                    .i = 0.3f,//0.3f
                                    .d = 0.0f,
                                    .i_limit = 0.0f,
                                    .out_limit = 200.0f,
                                    .d_cutoff_freq = -1.0f,
                                    .cycle = true,
                                },
                            .in_cutoff_freq = -1.0f,

                            .out_cutoff_freq = -1.0f,

                        },
                        Component::PosActuator::Param{
                            .speed =
                                {
                                    .k = 0.005f,
                                    .p = 1.0f,//1.0f,
                                    .i = 0.4f,
                                    .d = 0.001f,
                                    .i_limit = 0.25f,
                                    .out_limit = 0.4f,
                                    .d_cutoff_freq = -1.0f,
                                    .cycle = false,
                                },
                            .position =
                                {
                                    .k = 150.0f,
                                    .p = 1.0f,
                                    .i = 0.4f,
                                    .d = 0.0f,
                                    .i_limit = 0.0f,
                                    .out_limit = 200.0f,
                                    .d_cutoff_freq = -1.0f,
                                    .cycle = true,
                                },
                            .in_cutoff_freq = -1.0f,

                            .out_cutoff_freq = -1.0f,

                        },
                        Component::PosActuator::Param{
                            .speed =
                                {
                                    .k = 0.005f,
                                    .p = 1.0,//1.0f,
                                    .i = 0.4f,
                                    .d = 0.001f,
                                    .i_limit = 0.25f,
                                    .out_limit = 0.4f,
                                    .d_cutoff_freq = -1.0f,
                                    .cycle = false,
                                },
                            .position =
                                {
                                    .k = 150.0f,
                                    .p = 1.0f,
                                    .i = 0.4f,
                                    .d = 0.0f,
                                    .i_limit = 0.0f,
                                    .out_limit = 200.0f,
                                    .d_cutoff_freq = -1.0f,
                                    .cycle = true,
                                },
                            .in_cutoff_freq = -1.0f,

                            .out_cutoff_freq = -1.0f,

                        },
                        Component::PosActuator::Param{
                            .speed =
                                {
                                    .k = 0.005f,
                                    .p = 1.0f,//1.0f,
                                    .i = 0.4f,//0.3f
                                    .d = 0.001f,
                                    .i_limit = 0.25f,
                                    .out_limit = 0.4f,
                                    .d_cutoff_freq = -1.0f,
                                    .cycle = false,
                                },
                            .position =
                                {
                                    .k = 150.0f,
                                    .p = 1.0f,
                                    .i = 0.4f,//0.3f
                                    .d = 0.0f,
                                    .i_limit = 0.0f,
                                    .out_limit = 200.0f,
                                    .d_cutoff_freq = -1.0f,
                                    .cycle = true,
                                },
                            .in_cutoff_freq = -1.0f,

                            .out_cutoff_freq = -1.0f,
                        },

                    },
                .speed_param =
                    {
                        Component::SpeedActuator::Param{
                            .speed =
                                {
                                    .k = 0.0001f,//0.0001
                                    .p = 2.0f,
                                    .i = 0.0f,
                                    .d = 0.0f,
                                    .i_limit = 1.0f,
                                    .out_limit = 1.0f,
                                    .d_cutoff_freq = -1.0f,
                                    .cycle = false,
                                },

                            .in_cutoff_freq = -1.0f,

                            .out_cutoff_freq = -1.0f,

                        },
                        Component::SpeedActuator::Param{
                            .speed =
                                {
                                    .k = 0.0001f,//0.0001
                                    .p = 2.0f,
                                    .i = 0.0f,
                                    .d = 0.0f,
                                    .i_limit = 1.0f,
                                    .out_limit = 1.0f,
                                    .d_cutoff_freq = -1.0f,
                                    .cycle = false,
                                },

                            .in_cutoff_freq = -1.0f,

                            .out_cutoff_freq = -1.0f,
                        },
                        Component::SpeedActuator::Param{
                            .speed =
                                {
                                    .k = 0.0001f,
                                    .p = 2.0f,
                                    .i = 0.0f,
                                    .d = 0.0f,
                                    .i_limit = 1.0f,
                                    .out_limit = 1.0f,
                                    .d_cutoff_freq = -1.0f,
                                    .cycle = false,
                                },

                            .in_cutoff_freq = -1.0f,

                            .out_cutoff_freq = -1.0f,
                        },
                        Component::SpeedActuator::Param{
                            .speed =
                                {
                                    .k = 0.0001f,
                                    .p = 2.0f,
                                    .i = 0.0f,
                                    .d = 0.0f,
                                    .i_limit = 1.0f,
                                    .out_limit = 1.0f,
                                    .d_cutoff_freq = -1.0f,
                                    .cycle = false,
                                },

                            .in_cutoff_freq = -1.0f,

                            .out_cutoff_freq = -1.0f,
                        },
                    },
              .mech_zero = {0.448689401,3.6010201,3.14389372,3.13315582},
              //.mech_zero = {2.10860233,5.35004959,4.736166, 4.7754863},
                .pos_motor_param =
                    {
                        Device::RMMotor::Param{
                            .id_feedback = 0x206,
                            .id_control = GM6020_CTRL_ID_BASE,
                            .model = Device::RMMotor::MOTOR_GM6020,
                            .can = BSP_CAN_2,
                            .reverse = false,
                        },
                        Device::RMMotor::Param{
                            .id_feedback = 0x20A,
                            .id_control = GM6020_CTRL_ID_EXTAND,
                            .model = Device::RMMotor::MOTOR_GM6020,
                            .can = BSP_CAN_2,
                            .reverse = false,
                        },
                        Device::RMMotor::Param{
                            .id_feedback = 0x208,
                            .id_control = GM6020_CTRL_ID_BASE,
                            .model = Device::RMMotor::MOTOR_GM6020,
                            .can = BSP_CAN_2,
                            .reverse = false,
                        },
                        Device::RMMotor::Param{
                            .id_feedback = 0x205,
                            .id_control = GM6020_CTRL_ID_BASE,
                            .model = Device::RMMotor::MOTOR_GM6020,
                            .can = BSP_CAN_2,
                            .reverse = false,
                        },
                    },
                .speed_motor_param =
                    {
                        Device::RMMotor::Param{
                            .id_feedback = 0x203,
                            .id_control = M3508_M2006_CTRL_ID_BASE,
                            .model = Device::RMMotor::MOTOR_M3508,
                            .can = BSP_CAN_1,
                            .reverse = true,
                        },
                        Device::RMMotor::Param{
                            .id_feedback = 0x201,
                            .id_control = M3508_M2006_CTRL_ID_BASE,
                            .model = Device::RMMotor::MOTOR_M3508,
                            .can = BSP_CAN_1,
                            .reverse = true,
                        },
                        Device::RMMotor::Param{
                            .id_feedback = 0x202,
                            .id_control = M3508_M2006_CTRL_ID_BASE,
                            .model = Device::RMMotor::MOTOR_M3508,
                            .can = BSP_CAN_1,
                            .reverse = true,
                        },
                        Device::RMMotor::Param{
                            .id_feedback = 0x205,
                            .id_control = M3508_M2006_CTRL_ID_EXTAND,
                            .model = Device::RMMotor::MOTOR_M3508,
                            .can = BSP_CAN_1,
                            .reverse = true,
                        },
                    },
            },
        .gimbal =
            {
                .ff =
                    {
                        /* GIMBAL_CTRL_PIT_FEEDFORWARD */
                        .a = 0.0f,
                        .b = 0.0f,
                        .c = -0.05f,
                        .max = 0.1f,
                        .min = -0.2f,
                    }, /* ff */

                .st =
                    {
                        /* GIMBAL_CTRL_YAW_SELF_TUNING */
                        .a = 0.0677f,
                        .b = 0.1653f,
                        .c = 0.3379f,
                        .max = 0.37f,
                        .min = 0.29f,
                    }, /* st */

                .yaw_actr =
                    {
                        .speed =
                            {
                                /* GIMBAL_CTRL_YAW_OMEGA_IDX */
                                .k = 0.4f,
                                .p = 1.f,
                                .i = 0.5f,
                                .d = 0.f,
                                .i_limit = 0.2f,
                                .out_limit = 1.0f,
                                .d_cutoff_freq = -1.0f,
                                .cycle = false,
                            },

                        .position =
                            {
                                /* GIMBAL_CTRL_YAW_ANGLE_IDX */
                                .k = 20.0f,
                                .p = 1.0f,
                                .i = 0.0f,
                                .d = 0.0f,
                                .i_limit = 0.0f,
                                .out_limit = 10.0f,
                                .d_cutoff_freq = -1.0f,
                                .cycle = true,
                            },

                        .in_cutoff_freq = -1.0f,

                        .out_cutoff_freq = -1.0f,
                    },
                .pit_actr =
                    {
                        .speed =
                            {
                                /* GIMBAL_CTRL_PIT_OMEGA_IDX */
                                .k = 0.4f,
                                .p = 0.9f,//1.0f,
                                .i = 0.6f,//0.6f
                                .d = 0.0f,
                                .i_limit = 0.7f,
                                .out_limit = 1.0f,
                                .d_cutoff_freq = -1.0f,
                                .cycle = false,
                            },

                        .position =
                            {
                                /* GIMBAL_CTRL_PIT_ANGLE_IDX */
                                .k = 10.0f,
                                .p = 1.1f,//1.0f,
                                .i = 0.0f,
                                .d = 0.0f,
                                .i_limit = 0.0f,
                                .out_limit = 10.0f,
                                .d_cutoff_freq = -1.0f,
                                .cycle = true,
                            },

                        .in_cutoff_freq = -1.0f,

                        .out_cutoff_freq = -1.0f,
                    },

                .yaw_ai_actr =
                    {
                        .speed =
                            {
                                /* GIMBAL_CTRL_YAW_OMEGA_IDX */
                                .k = 0.4f,
                                .p = 1.f,
                                .i = 0.5f,
                                .d = 0.f,
                                .i_limit = 0.2f,
                                .out_limit = 1.0f,
                                .d_cutoff_freq = -1.0f,
                                .cycle = false,
                            },

                        .position =
                            {
                                /* GIMBAL_CTRL_YAW_ANGLE_IDX */
                                .k = 20.0f,
                                .p = 1.0f,
                                .i = 0.0f,
                                .d = 0.0f,
                                .i_limit = 0.0f,
                                .out_limit = 10.0f,
                                .d_cutoff_freq = -1.0f,
                                .cycle = true,
                            },

                        .in_cutoff_freq = -1.0f,

                        .out_cutoff_freq = -1.0f,
                    },
                .pit_ai_actr =
                    {
                        .speed =
                            {
                                /* GIMBAL_CTRL_PIT_OMEGA_IDX */
                                .k = 0.4f,
                                .p = 0.9f,//1.0f,
                                .i = 0.6f,//0.6f
                                .d = 0.0f,
                                .i_limit = 0.7f,
                                .out_limit = 1.0f,
                                .d_cutoff_freq = -1.0f,
                                .cycle = false,
                            },

                        .position =
                            {
                                /* GIMBAL_CTRL_PIT_ANGLE_IDX */
                                .k = 10.0f,
                                .p = 1.1f,//1.0f,
                                .i = 0.0f,
                                .d = 0.0f,
                                .i_limit = 0.0f,
                                .out_limit = 10.0f,
                                .d_cutoff_freq = -1.0f,
                                .cycle = true,
                            },

                        .in_cutoff_freq = -1.0f,

                        .out_cutoff_freq = -1.0f,
                    },

                .yaw_motor =
                    {
                        .id_feedback = 0x209,
                        .id_control = GM6020_CTRL_ID_EXTAND,
                        .model = Device::RMMotor::MOTOR_GM6020,
                        .can = BSP_CAN_2,
                        .reverse=false,
                    },

                .pit_motor =
                    {
                        .id_feedback = 0x20B,
                        .id_control = GM6020_CTRL_ID_EXTAND,
                        .model = Device::RMMotor::MOTOR_GM6020,
                        .can = BSP_CAN_2,
                        .reverse = true,
                    },

                .mech_zero =
                    {
                        //.yaw = 2.08314586f,
                        .yaw = 3.66237926f,
                        .pit = M_2PI - 5.4602046f,
                        .rol = 0.0f,
                    },

                .limit =
                    {
                        .pitch_max = M_2PI - 5.1f,
                        .pitch_min = M_2PI - 5.8f,
                        .yaw_max = 0.0f,
                        .yaw_min = 0.0f,
                    },

                .EVENT_MAP = {Component::CMD::EventMapItem{
                                  Component::CMD::CMD_EVENT_LOST_CTRL,
                                  Module::Gimbal::SET_MODE_RELAX},
                              Component::CMD::EventMapItem{
                                  Device::DR16::DR16_SW_R_POS_TOP,
                                  Module::Gimbal::ABSOLUTE},
                              Component::CMD::EventMapItem{
                                  Device::DR16::DR16_SW_R_POS_TOP,
                                  Module::Gimbal::SET_MODE_ABSOLUTE},
                              Component::CMD::EventMapItem{
                                  Device::DR16::DR16_SW_R_POS_MID,
                                  Module::Gimbal::SET_MODE_ABSOLUTE},
                              Component::CMD::EventMapItem{
                                  Device::DR16::DR16_SW_R_POS_BOT,
                                  Module::Gimbal::SET_MODE_ABSOLUTE},
                              Component::CMD::EventMapItem{
                                  Device::DR16::KEY_R_PRESS,
                                  Module::Gimbal::SET_MODE_AUTO_AIM},
                              Component::CMD::EventMapItem{
                                  Device::DR16::KEY_R_RELEASE,
                                  Module::Gimbal::SET_MODE_ABSOLUTE}},

            },
    .launcher = {
    .num_trig_tooth = 10.0f,
    .trig_gear_ratio = 36.0f * 2.5f,
    .fric_radius = 0.03f,
    .cover_open_duty = 0.125f,
    .cover_close_duty = 0.075f,
    .model = Module::RMLauncher::LAUNCHER_MODEL_17MM,
    .default_bullet_speed = 25.0f,
        .min_launch_delay = static_cast<uint32_t>(1000.0f / 20.0f),
        // .allow_reverse = false,
        // .fric_speed = 3000.0f,

        .trig_actr = {
            Component::PosActuator::Param{
                .speed = {
                .k = 3.0f,
                .p = 1.0f,
                .i = 0.5f,
                .d = 0.0f,
                .i_limit = 0.5f,
                .out_limit = 1.0f,
                .d_cutoff_freq = -1.0f,
                .cycle = false,
                },

                .position = {
                .k = 1.5f,
                .p = 1.0f,
                .i = 0.0f,
                .d = 0.0f,
                .i_limit = 1.0f,
                .out_limit = 1.0f,
                .d_cutoff_freq = -1.0f,
                .cycle = true,
                },

                .in_cutoff_freq = -1.0f,

                .out_cutoff_freq = -1.0f,
            },
        },

        .fric_actr = {
           Component::SpeedActuator::Param{
            .speed = {
            .k = 0.0003f,
            .p = 1.8f,
            .i = 0.0f,
            .d = 0.0f,
            .i_limit = 0.2f,
            .out_limit = 1.0f,
            .d_cutoff_freq = -1.0f,
            .cycle = false,
            },

            .in_cutoff_freq = -1.0f,

            .out_cutoff_freq = -1.0f,
            },
           Component::SpeedActuator::Param{
            .speed = {
            .k = 0.0003f,
            .p = 1.8f,
            .i = 0.0f,
            .d = 0.0f,
            .i_limit = 0.2f,
            .out_limit = 1.0f,
            .d_cutoff_freq = -1.0f,
            .cycle = false,
            },

            .in_cutoff_freq = -1.0f,

            .out_cutoff_freq = -1.0f,
        },
        },

        .trig_param = {
        Device::RMMotor::Param{
            .id_feedback = 0x204,
            .id_control = M3508_M2006_CTRL_ID_BASE,
            .model = Device::RMMotor::MOTOR_M2006,
            .can = BSP_CAN_1,
            .reverse = true,
        }
        },

        .fric_param = {
        Device::RMMotor::Param{
            .id_feedback = 0x206,
            .id_control = M3508_M2006_CTRL_ID_EXTAND,
            .model = Device::RMMotor::MOTOR_M3508,
            .can = BSP_CAN_1,
            .reverse=false,
        },
        Device::RMMotor::Param{
            .id_feedback = 0x207,
            .id_control = M3508_M2006_CTRL_ID_EXTAND,
            .model = Device::RMMotor::MOTOR_M3508,
            .can = BSP_CAN_1,
            .reverse=false,
        },
        },

        .EVENT_MAP = {
        Component::CMD::EventMapItem{
            Component::CMD::CMD_EVENT_LOST_CTRL,
            Module::RMLauncher::CHANGE_FIRE_MODE_RELAX
        },
        Component::CMD::EventMapItem{
            Device::DR16::DR16_SW_R_POS_TOP,
            Module::RMLauncher::CHANGE_FIRE_MODE_SAFE
        },
        Component::CMD::EventMapItem{
            Device::DR16::DR16_SW_R_POS_MID,
            Module::RMLauncher::CHANGE_FIRE_MODE_LOADED
        },
        // Component::CMD::EventMapItem{
        //     Device::DR16::DR16_SW_R_POS_MID,
        //     Module::Launcher::LAUNCHER_STOP_TRIG
        // },
        Component::CMD::EventMapItem{
            Device::DR16::DR16_SW_R_POS_BOT,
            Module::RMLauncher::CHANGE_FIRE_MODE_LOADED,
        },
        Component::CMD::EventMapItem{
            Device::DR16::DR16_SW_R_POS_BOT,
            Module::RMLauncher::LAUNCHER_START_FIRE,
        },
        // Component::CMD::EventMapItem{
        //     Device::DR16::DR16_SW_R_POS_BOT,
        //     Module::Launcher::CHANGE_TRIG_MODE_BURST,
        // },
        Component::CMD::EventMapItem{
            Device::DR16::KEY_L_PRESS,
            Module::RMLauncher::CHANGE_TRIG_MODE_BURST
        },
        Component::CMD::EventMapItem{
        Device::DR16::KEY_L_RELEASE,
        Module::RMLauncher::LAUNCHER_STOP_TRIG
        },
        Component::CMD::EventMapItem{
            Device::DR16::KEY_G,
            Module::RMLauncher::CHANGE_TRIG_MODE
        },

    },
  }, /* launcher */
  .bmi088_rot =
            {
                .rot_mat =
                    {
                        {+0, -1, +0},
                        {+1, +0, +0},
                        {+0, +0, +1},
                    },
            },
  .cap = {
    .can = BSP_CAN_2,
  },
};
/* clang-format on */

void robot_init() {
  System::Start<Robot::HelmInfantry, Robot::HelmInfantry::Param>(param, 500.0f);
}
