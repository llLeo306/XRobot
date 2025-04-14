#include "dev_aim.hpp"
using namespace Device;

#define AI_LEN_RX_BUFF (sizeof(AIM::RefForAI_t))
#define AI_LEN_TX_BUFF (sizeof(AIM::TranToAI_t))

static uint8_t rxbuf[AI_LEN_RX_BUFF];
static uint8_t txbuf[AI_LEN_TX_BUFF];

/* clang-format off */
AIM::AIM()
    : event_(Message::Event::FindEvent("cmd_event")),
      cmd_tp_("cmd_ai"),
      data_ready_(false)
{

  auto rx_cplt_callback = [](void *arg) {
    AIM *aim = static_cast<AIM *>(arg);
    aim->data_ready_.Post();
  };

  bsp_uart_register_callback(BSP_UART_AI, BSP_UART_RX_CPLT_CB, rx_cplt_callback,
                             this);

  Component::CMD::RegisterController(this->cmd_tp_);
 auto ai_thread = [](AIM *aim) {
    auto ref_sub = Message::Subscriber<Device::Referee::Data>("referee");

    auto eulr_sub = Message::Subscriber<Component::Type::Eulr>("imu_eulr");

     while (1)
     {
        /* 接收imu和裁判系统数据 */
        eulr_sub.DumpData(aim->eulr_);

        /* 接收裁判系统数据 */
        if (ref_sub.DumpData(aim->raw_ref_)) {
        aim->PraseRef();
        aim->PackRef();
        }

        /* 接收上位机数据 */
        aim->StartRecv();
        if (aim->data_ready_.Wait(0))
        {
            aim->PraseHost();
            aim->PackCMD();
        }

         //aim->ai_tp_.Publish(aim->cmd_for_ref_);

        //发送数据给上位机
        aim->PackMCU();
        aim->StartTran();

        System::Thread::Sleep(2);
     }
    };
    this->thread_.Create(ai_thread, this, "aim_thread", DEVICE_AI_TASK_STACK_DEPTH,
                       System::Thread::REALTIME);
}

/* --------------------------------------------------------------------------------------------- */
/* ----------------------------------------通讯方面---------------------------------------------- */
/* --------------------------------------------------------------------------------------------- */
double convert_to_0_to_2pi(double theta_prime) {
  double theta = fmod(theta_prime + 2 * M_PI, 2 * M_PI);
  return theta;
}

bool AIM::PackCMD() {
  /* 确保遥控器开关最高控制权，关遥控器即断控 */
  if (!Component::CMD::Online()) {
    return false;
  }

    if(from_host_.v_yaw == 0 && from_host_.pitch == 0)
    {
      return 0;
    }

  /* 控制源：AI */
  if (Component::CMD::GetCtrlSource() == Component::CMD::CTRL_SOURCE_AI) {
    /* OP控制模式，用于鼠标右键自瞄 */
    if (Component::CMD::GetCtrlMode() == Component::CMD::CMD_OP_CTRL) {

  if(from_host_.header == 0xA5)
  {
    this->to_cmd_.yaw = from_host_.yaw;
    this->to_cmd_.pitch = from_host_.pitch;
    this->to_cmd_.roll = this->eulr_.rol;
  }

      memcpy(&(this->cmd_.gimbal.eulr), &(this->to_cmd_),
             sizeof(this->cmd_.gimbal.eulr));

      this->cmd_.ctrl_source = Component::CMD::CTRL_SOURCE_AI;

      this->cmd_tp_.Publish(this->cmd_);
      this->cmd_tp_.Publish(this->cmd_);
      this->cmd_tp_.Publish(this->cmd_);
    }
  }
  return true;
}

bool AIM::PraseHost() {
 if (Component::CRC16::Verify(rxbuf, sizeof(this->from_host_))) {
    this->cmd_.online = true;
    this->last_online_time_ = bsp_time_get_ms();
    memcpy(&(this->from_host_), rxbuf, sizeof(this->from_host_));
    memset(rxbuf, 0, AI_LEN_RX_BUFF);
    return true;
  }
 return false;
}

double convert_to_minus_pi_to_pi(double theta) {
  while (theta > M_PI) {
    theta -= 2 * M_PI;
  }
  while (theta < -M_PI) {
    theta += 2 * M_PI;
  }
  return theta;
}

bool AIM::PackMCU()
{
    this->to_host_.header = 0x5A;
    this->to_host_.detect_color = 0;
    this->to_host_.reset_tracker = 0;
    this->to_host_.current_v = 24;
    this->to_host_.yaw = this->eulr_.yaw;
    this->to_host_.pitch = this->eulr_.pit;
    this->to_host_.roll = this->eulr_.rol;
    this->to_host_.aim_x = this->from_host_.x;
    this->to_host_.aim_y = this->from_host_.y;
    this->to_host_.aim_z = this->from_host_.z;
    this->to_host_.checksum = Component::CRC16::Calculate(
      reinterpret_cast<const uint8_t *>(&(this->to_host_)),
      sizeof(this->to_host_) - sizeof(uint16_t), CRC16_INIT);

    return true;
}
bool AIM::StartTran()
{
    size_t len = sizeof(this->to_host_);

    void *src = NULL;
    src = &(this->to_host_);
    memcpy(txbuf, src, len);
    return bsp_uart_transmit(BSP_UART_AI, txbuf, len,false) == BSP_OK;
}

bool AIM::StartRecv()
{
  return bsp_uart_receive(BSP_UART_AI, rxbuf, sizeof(rxbuf), false) == BSP_OK;
}

void AIM::PraseRef() {
#if RB_HERO
  this->ref_.ball_speed = BULLET_SPEED_LIMIT_42MM
#else
  this->ref_.ball_speed = BULLET_SPEED_LIMIT_17MM;
#endif

                          this->ref_.max_hp =
      this->raw_ref_.robot_status.max_hp;

  this->ref_.hp = this->raw_ref_.robot_status.remain_hp;

  if (this->raw_ref_.robot_status.robot_id < Referee::REF_BOT_BLU_HERO) {
    this->ref_.team = AI_TEAM_RED;
  } else {
    this->ref_.team = AI_TEAM_BLUE;
  }
  this->ref_.status = this->raw_ref_.status;

  switch (this->raw_ref_.game_status.game_type) {
    case Referee::REF_GAME_TYPE_RMUC:
      this->ref_.game_type = AI_RACE_RMUC;
      break;
    case Referee::REF_GAME_TYPE_RMUT:
      this->ref_.game_type = AI_RACE_RMUT;
      break;
    case Referee::REF_GAME_TYPE_RMUL_3V3:
      this->ref_.game_type = AI_RACE_RMUL3;
      break;
    case Referee::REF_GAME_TYPE_RMUL_1V1:
      this->ref_.game_type = AI_RACE_RMUL1;
      break;
    default:
      return;
  }

  switch (this->raw_ref_.robot_status.robot_id % 100) {
    case Referee::REF_BOT_RED_HERO:
      this->ref_.robot_id = AI_ARM_HERO;
      break;
    case Referee::REF_BOT_RED_ENGINEER:
      this->ref_.robot_id = AI_ARM_ENGINEER;
      break;
    case Referee::REF_BOT_RED_DRONE:
      this->ref_.robot_id = AI_ARM_DRONE;
      break;
    case Referee::REF_BOT_RED_SENTRY:
      this->ref_.robot_id = AI_ARM_SENTRY;
      break;
    case Referee::REF_BOT_RED_RADER:
      this->ref_.robot_id = AI_ARM_RADAR;
      break;
    default:
      this->ref_.robot_id = AI_ARM_INFANTRY;
  }

  this->ref_.game_progress = this->raw_ref_.game_status.game_progress;

  if (this->raw_ref_.robot_status.robot_id < 100) {
    this->ref_.base_hp = this->raw_ref_.game_robot_hp.red_base;
    this->ref_.outpost_hp = this->raw_ref_.game_robot_hp.red_outpose;
    this->ref_.hp = this->raw_ref_.game_robot_hp.red_7;
    // this->ref_.hp = 400;

  } else {
    this->ref_.base_hp = this->raw_ref_.game_robot_hp.blue_base;
    this->ref_.outpost_hp = this->raw_ref_.game_robot_hp.blue_outpose;
    this->ref_.hp = this->raw_ref_.game_robot_hp.blue_7;
  }
  this->ref_.coin_num = this->raw_ref_.bullet_remain.coin_remain;
  this->ref_.pos_x = this->raw_ref_.robot_pos.x;
  this->ref_.pos_y = this->raw_ref_.robot_pos.y;
  this->ref_.pos_angle = this->raw_ref_.robot_pos.angle;

  this->ref_.target_pos_x = this->raw_ref_.client_map.position_x;
  this->ref_.target_pos_y = this->raw_ref_.client_map.position_y;

  if (this->raw_ref_.robot_damage.damage_type == 0) {
    this->ref_.damaged_armor_id = this->raw_ref_.robot_damage.armor_id;
  }
}

bool AIM::PackRef() {

  return true;
}
