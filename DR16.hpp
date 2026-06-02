#pragma once

/* clang-format off */
/* === MODULE MANIFEST ===
module_name: DR16
module_description: Receiver parsing
constructor_args:
  - CMD: '@cmd'
  - task_stack_depth_uart: 2048
  - thread_priority_uart: LibXR::Thread::Priority::HIGH
required_hardware: dr16 dma uart
=== END MANIFEST === */
// clang-format on

#include <algorithm>
#include <cstdint>
#include <cstring>

#include "CMD.hpp"
#include "app_framework.hpp"
#include "thread.hpp"
#include "timebase.hpp"
#include "uart.hpp"

/**
 * @brief DR16遥控器通道值范围定义
 */
#define DR16_CH_VALUE_MIN (364u)  /* 通道最小值 */
#define DR16_CH_VALUE_MID (1024u) /* 通道中间值 */
#define DR16_CH_VALUE_MAX (1684u) /* 通道最大值 */

/**
 * @class DR16
 * @brief DR16遥控器数据解析类
 * @details 负责接收和解析DR16遥控器的数据，包括摇杆���拨杆和按键等信息
 */
class DR16 : public LibXR::Application {
 public:
  /**
   * @brief 拨杆开关位置枚举
   */
  enum class SwitchPos : uint8_t {
    DR16_SW_L_POS_TOP = 0x00,
    DR16_SW_L_POS_BOT = 0x01,
    DR16_SW_L_POS_MID = 0x02,
    DR16_SW_R_POS_TOP = 0x03,
    DR16_SW_R_POS_BOT = 0x04,
    DR16_SW_R_POS_MID = 0x05,
    DR16_SW_POS_NUM = 6
  };

  /**
   * @brief 按键枚举    SET_MODE_RELAX,
    SET_MODE_FOLLOW,
    SET_MODE_ROTOR,
    SET_MODE_INDENPENDENT,
   */
  enum class Key : uint8_t {
    KEY_W = static_cast<uint8_t>(SwitchPos::DR16_SW_POS_NUM),
    KEY_S,
    KEY_A,
    KEY_D,
    KEY_SHIFT,
    KEY_CTRL,
    KEY_Q,
    KEY_E,
    KEY_R,
    KEY_F,
    KEY_G,
    KEY_Z,
    KEY_X,
    KEY_C,
    KEY_V,
    KEY_B,
    KEY_L_PRESS,
    KEY_R_PRESS,
    KEY_L_RELEASE,
    KEY_R_RELEASE,
    KEY_NUM,
  };

  /**
   * @brief 计算Shift组合键的编码值
   * @param key 基础按键
   * @return Shift组合键的编码值
   */
  constexpr uint32_t ShiftWith(Key key) {
    return static_cast<uint8_t>(key) + 1 * static_cast<uint8_t>(Key::KEY_NUM);
  }

  /**
   * @brief 计算Ctrl组合键的编码值
   * @param key 基础按键
   * @return Ctrl组合键的编码值
   */
  constexpr uint32_t CtrlWith(Key key) {
    return static_cast<uint8_t>(key) + 2 * static_cast<uint8_t>(Key::KEY_NUM);
  }

  /**
   * @brief 计算Shift+Ctrl组合键的编码值
   * @param key 基础按键
   * @return Shift+Ctrl组合键的编码值
   */
  constexpr uint32_t ShiftCtrlWith(Key key) {
    return static_cast<uint8_t>(key) + 3 * static_cast<uint8_t>(Key::KEY_NUM);
  }

  constexpr uint32_t RawValue(Key key) {
    if (key >= Key::KEY_NUM) {
      return 0;
    }
    return 1 << (static_cast<uint8_t>(key) - static_cast<uint8_t>(Key::KEY_W));
  }

  typedef struct __attribute__((packed)) {
    uint16_t ch_r_x;
    uint16_t ch_r_y;
    uint16_t ch_l_x;
    uint16_t ch_l_y;
    uint8_t sw_r;
    uint8_t sw_l;
    int16_t x;
    int16_t y;
    int16_t z;
    uint8_t press_l;
    uint8_t press_r;
    uint16_t key;
    uint16_t res;
  } Data;

  /**
   * @brief DR16构造函数
   * @param hw 硬件容器引用
   * @param app 应用管理器引用
   * @param cmd 控制命令对象引用
   * @param task_stack_depth_uart UART任务栈深度
   * @param cmd_data_tp_name CMD数据主题名称
   */
  DR16(LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app, CMD& cmd,
       uint32_t task_stack_depth_uart,
       LibXR::Thread::Priority thread_priority_uart =
           LibXR::Thread::Priority::MEDIUM)
      : cmd_(&cmd),
        uart_(hw.Find<LibXR::UART>("uart_dr16")),
        sem_(0),
        op_(sem_, 4) {
    uart_->SetConfig({100000, LibXR::UART::Parity::EVEN, 8, 1});
    /* 创建UART线程 */
    thread_uart_.Create(this, ThreadDr16, "uart_dr16", task_stack_depth_uart,
                        thread_priority_uart);
    app.Register(*this);
  }

  /**
   * @brief 获取 DR16 的事件处理器
   * @return LibXR::Event& 事件处理器的引用
   */
  LibXR::Event& GetEvent() { return dr16_event_; }

  /**
   * @brief 监控函数重写
   */
  void OnMonitor() override {}

  /**
   * @brief DR16 UART读取线程函数
   * @param dr16 DR16实例指针
   */
  static void ThreadDr16(DR16* dr16) {
    dr16->uart_->read_port_->Reset();

    constexpr std::size_t RX_BUFFER_SIZE = 18;
    uint8_t rx_buffer[RX_BUFFER_SIZE] = {0};
    CMD::Data rc_data;

    auto last_time = LibXR::Timebase::GetMilliseconds();
    while (1) {
      if (dr16->uart_->Read({rx_buffer, RX_BUFFER_SIZE}, dr16->op_) ==
          LibXR::ErrorCode::OK) {
        if (dr16->ParseRC(rx_buffer, rc_data) == LibXR::ErrorCode::OK) {
          dr16->last_time_ = LibXR::Timebase::GetMilliseconds();
          dr16->cmd_->FeedRC(CMD::RCInputSource::RC_INPUT_DR16, rc_data);
        } else {
          dr16->uart_->read_port_->Reset();
          LibXR::Memory::FastSet(rx_buffer, 0, RX_BUFFER_SIZE);
        }
      }
      dr16->CheckoutOffline();
      LibXR::Thread::SleepUntil(last_time, 5);
    }
  }

  /**
   * @brief 解析遥控器数据并转换为控制命令
   */
  /**
   * @brief 解析 DBUS 原始数据并生成控制指令
   * @param raw_data 18字节的原始接收缓冲 (来自 ThreadDr16 的 rx_buffer)
   * @param output_data 解析后的 CMD 数据 (用于提交给云台)
   * @return true 解析成功, false 数据无效
   */
  LibXR::ErrorCode ParseRC(const uint8_t* raw_data, CMD::Data& output_data) {
    if (!raw_data) {
      return LibXR::ErrorCode::PTR_NULL;
    };

    Data curr_rc{};

    curr_rc.ch_r_x = ((raw_data[0] | raw_data[1] << 8) & 0x07FF);
    curr_rc.ch_r_y = ((raw_data[1] >> 3 | raw_data[2] << 5) & 0x07FF);
    curr_rc.ch_l_x =
        ((raw_data[2] >> 6 | raw_data[3] << 2 | raw_data[4] << 10) & 0x07FF);
    curr_rc.ch_l_y = ((raw_data[4] >> 1 | raw_data[5] << 7) & 0x07FF);

    curr_rc.sw_r = ((raw_data[5] >> 4) & 0x0003);  // bits 4-5
    curr_rc.sw_l = ((raw_data[5] >> 6) & 0x0003);  // bits 6-7

    curr_rc.x = static_cast<int16_t>(raw_data[6] | raw_data[7] << 8);
    curr_rc.y = static_cast<int16_t>(raw_data[8] | raw_data[9] << 8);
    curr_rc.z = static_cast<int16_t>(raw_data[10] | raw_data[11] << 8);

    curr_rc.press_l = raw_data[12];
    curr_rc.press_r = raw_data[13];

    curr_rc.key = static_cast<uint16_t>(raw_data[14] | raw_data[15] << 8);

    curr_rc.res = static_cast<uint16_t>(raw_data[16] | raw_data[17] << 8);

#ifndef NDEBUG
    this->data_review_ = curr_rc;
#endif

    if (curr_rc.ch_l_x < DR16_CH_VALUE_MIN ||
        curr_rc.ch_l_x > DR16_CH_VALUE_MAX ||
        curr_rc.ch_l_y < DR16_CH_VALUE_MIN ||
        curr_rc.ch_l_y > DR16_CH_VALUE_MAX ||
        curr_rc.ch_r_x < DR16_CH_VALUE_MIN ||
        curr_rc.ch_r_x > DR16_CH_VALUE_MAX ||
        curr_rc.ch_r_y < DR16_CH_VALUE_MIN ||
        curr_rc.ch_r_y > DR16_CH_VALUE_MAX) {
      return LibXR::ErrorCode::CHECK_ERR;
    }

    if (curr_rc.sw_l == 0 || curr_rc.sw_r == 0) {
      return LibXR::ErrorCode::CHECK_ERR;
    }

    output_data = CMD::Data();

    if (curr_rc.sw_l != this->last_data_.sw_l) {
      this->dr16_event_.Active(
          static_cast<uint32_t>(SwitchPos::DR16_SW_L_POS_TOP) + curr_rc.sw_l -
          1);
    }
    if (curr_rc.sw_r != this->last_data_.sw_r) {
      this->dr16_event_.Active(
          static_cast<uint32_t>(SwitchPos::DR16_SW_R_POS_TOP) + curr_rc.sw_r -
          1);
    }

    uint32_t modifier_offset = 0;

    if (curr_rc.key & RawValue(Key::KEY_SHIFT)) {
      modifier_offset += static_cast<uint32_t>(Key::KEY_NUM);
    }
    if (curr_rc.key & RawValue(Key::KEY_CTRL)) {
      modifier_offset += 2 * static_cast<uint32_t>(Key::KEY_NUM);
    }

    for (int i = 0; i < 16; i++) {
      if ((curr_rc.key & (1 << i)) && !(this->last_data_.key & (1 << i))) {
        this->dr16_event_.Active(static_cast<uint32_t>(Key::KEY_W) + i +
                                 modifier_offset);
      }
    }

    constexpr float FULL_RANGE =
        static_cast<float>(DR16_CH_VALUE_MAX - DR16_CH_VALUE_MIN);
    constexpr float INV_FULL_RANGE = 1.0f / FULL_RANGE;
    constexpr float MOUSE_SCALER = 20.0f / 32768.0f;

    if (curr_rc.press_l && !this->last_data_.press_l) {
      this->dr16_event_.Active(static_cast<uint32_t>(Key::KEY_L_PRESS));
    }
    if (!curr_rc.press_l && this->last_data_.press_l) {
      this->dr16_event_.Active(static_cast<uint32_t>(Key::KEY_L_RELEASE));
    }
    if (curr_rc.press_r && !this->last_data_.press_r) {
      this->dr16_event_.Active(static_cast<uint32_t>(Key::KEY_R_PRESS));
    }
    if (!curr_rc.press_r && this->last_data_.press_r) {
      this->dr16_event_.Active(static_cast<uint32_t>(Key::KEY_R_RELEASE));
    }

    output_data.chassis.x =
        2 * (static_cast<float>(curr_rc.ch_l_x) - DR16_CH_VALUE_MID) *
        INV_FULL_RANGE;
    output_data.chassis.y =
        2 * (static_cast<float>(curr_rc.ch_l_y) - DR16_CH_VALUE_MID) *
        INV_FULL_RANGE;
    output_data.chassis.z =
        -2 * (static_cast<float>(curr_rc.ch_r_x) - DR16_CH_VALUE_MID) *
        INV_FULL_RANGE;

    output_data.gimbal.yaw =
        -2 * (static_cast<float>(curr_rc.ch_r_x) - DR16_CH_VALUE_MID) *
        INV_FULL_RANGE;
    output_data.gimbal.pit =
        2 * (static_cast<float>(curr_rc.ch_r_y) - DR16_CH_VALUE_MID) *
        INV_FULL_RANGE;

    if (curr_rc.key & RawValue(Key::KEY_A)) {
      output_data.chassis.x -= 1.0f;
    }
    if (curr_rc.key & RawValue(Key::KEY_D)) {
      output_data.chassis.x += 1.0f;
    }
    if (curr_rc.key & RawValue(Key::KEY_S)) {
      output_data.chassis.y -= 1.0f;
    }
    if (curr_rc.key & RawValue(Key::KEY_W)) {
      output_data.chassis.y += 1.0f;
    }

    output_data.chassis.self_define = CMD::ChasStat::NONE;

    output_data.gimbal.pit += static_cast<float>(curr_rc.y) * MOUSE_SCALER;
    output_data.gimbal.yaw += -static_cast<float>(curr_rc.x) * MOUSE_SCALER;

    if (curr_rc.key & RawValue(Key::KEY_SHIFT) or
        curr_rc.res == DR16_CH_VALUE_MAX) {
      output_data.chassis.self_define = CMD::ChasStat::BOOST;
    }

    if (curr_rc.key & RawValue(Key::KEY_C) or
        curr_rc.res == DR16_CH_VALUE_MIN) {
      output_data.chassis.self_define = CMD::ChasStat::STRETCH;
    }

    output_data.chassis.x = std::clamp(output_data.chassis.x, -1.0f, 1.0f);
    output_data.chassis.y = std::clamp(output_data.chassis.y, -1.0f, 1.0f);
    output_data.chassis.z = std::clamp(output_data.chassis.z, -1.0f, 1.0f);

    output_data.launcher.isfire =
        (curr_rc.res == DR16_CH_VALUE_MIN) or (curr_rc.press_l == 1);

    output_data.chassis_online = true;
    output_data.gimbal_online = true;
    output_data.ctrl_source = CMD::ControlSource::CTRL_SOURCE_RC;

    this->last_data_ = curr_rc;

    return LibXR::ErrorCode::OK;
  }

  void Offline() {
    cmd_data_.chassis.x = 0;
    cmd_data_.chassis.y = 0;
    cmd_data_.chassis.z = 0;
    cmd_data_.chassis.self_define = CMD::ChasStat::NONE;

    cmd_data_.gimbal.yaw = 0;
    cmd_data_.gimbal.pit = 0;

    cmd_data_.launcher.isfire = false;

    cmd_data_.chassis_online = false;
    cmd_data_.gimbal_online = false;

    cmd_->FeedRC(CMD::RCInputSource::RC_INPUT_DR16, cmd_data_);
  }

#ifdef LIBXR_DEBUG_BUILD
  /**
   * @brief 用于调试的数据视图结构体（非位域）
   */
  struct DataView {
    uint16_t ch_r_x; /* 右摇杆X轴 */
    uint16_t ch_r_y; /* 右摇杆Y轴 */
    uint16_t ch_l_x; /* 左摇杆X轴 */
    uint16_t ch_l_y; /* 左摇杆Y轴 */
    uint8_t sw_r;    /* 右拨杆 */
    uint8_t sw_l;    /* 左拨杆 */
    int16_t x;       /* 鼠标X轴移动 */
    int16_t y;       /* 鼠标Y轴移动 */
    int16_t z;       /* 鼠标Z轴移动 */
    uint8_t press_l; /* 鼠标左键状态 */
    uint8_t press_r; /* 鼠标右键状态 */
    uint16_t key;    /* 键盘按键状态 */
    uint16_t res;    /* 保留字段 */
  };

  /**
   * @brief 将位域数据转换为普通结构体数据（调试用）
   * @param data_view 输出的数据视图
   * @param data 输入的位域数据
   */
  void DataviewToData(DataView& data_view, Data& data) {
    data_view.ch_r_x = data.ch_r_x;
    data_view.ch_r_y = data.ch_r_y;
    data_view.ch_l_x = data.ch_l_x;
    data_view.ch_l_y = data.ch_l_y;
    data_view.sw_r = data.sw_r;
    data_view.sw_l = data.sw_l;
    data_view.x = data.x;
    data_view.y = data.y;
    data_view.z = data.z;
    data_view.press_l = data.press_l;
    data_view.press_r = data.press_r;
    data_view.key = data.key;
    data_view.res = data.res;
  }
#endif

 private:
  CMD* cmd_; /* CMD模块指针 */

  Data last_data_{};     /* 上一帧数据 */
  CMD::Data cmd_data_{}; /* 命令数据 */
#ifndef NDEBUG
  Data data_review_; /* 命令数据预览 */
#endif
  LibXR::UART* uart_;                       /* UART接口指针 */
  LibXR::Event dr16_event_;                 /* 事件处理器 */
  LibXR::Thread thread_uart_;               /* UART线程 */
  LibXR::Semaphore sem_;                    /* 读操作信号量 */
  LibXR::ReadOperation op_;                 /* 读操作（阻塞型） */
  LibXR::MillisecondTimestamp last_time_{}; /* 上次接收时间 */

  /*--------------------------工具函数-------------------------------------------------*/
  void CheckoutOffline() {
    auto current_time = LibXR::Timebase::GetMilliseconds();
    if ((current_time - last_time_).ToMillisecond() > 100) {
      Offline();
    }
  }
};
