#pragma once

#include <memory>

/* clang-format off */
/* === MODULE MANIFEST V2 ===
module_description: DR16 遥控接收机解析模块：从 UART 接收 DBUS 数据并转换为 CMD 控制数据 / Module that parses DR16 receiver DBUS data from a UART and passes it to CMD as control data
depends:
- id: QDU-Robomaster/CMD
  ref: same-or-dev
=== END MANIFEST === */
// clang-format on

#include <algorithm>
#include <cstdint>
#include <cstring>

#include "CMD.hpp"
#include "thread.hpp"
#include "timebase.hpp"
#include "uart.hpp"

#define DR16_CH_VALUE_MIN (364u)   ///< 通道最小值 Minimum channel value
#define DR16_CH_VALUE_MID (1024u)  ///< 通道中间值 Middle channel value
#define DR16_CH_VALUE_MAX (1684u)  ///< 通道最大值 Maximum channel value

/**
 * @brief DR16 遥控接收机解析模块：从 UART 接收 DBUS 数据，转换为 CMD 控制数据
 *        并发出按键事件。
 *        DR16 receiver parsing Module that receives DBUS data from a UART, converts it
 *        to CMD control data and emits key events.
 */
class DR16
{
 public:
  /**
   * @brief 拨杆位置事件 ID。
   *        Switch position event IDs.
   */
  enum class SwitchPos : uint8_t
  {
    DR16_SW_L_POS_TOP = 0x00,  ///< 左拨杆上位 Left switch top position
    DR16_SW_L_POS_BOT = 0x01,  ///< 左拨杆下位 Left switch bottom position
    DR16_SW_L_POS_MID = 0x02,  ///< 左拨杆中位 Left switch middle position
    DR16_SW_R_POS_TOP = 0x03,  ///< 右拨杆上位 Right switch top position
    DR16_SW_R_POS_BOT = 0x04,  ///< 右拨杆下位 Right switch bottom position
    DR16_SW_R_POS_MID = 0x05,  ///< 右拨杆中位 Right switch middle position
    DR16_SW_POS_NUM = 6        ///< 拨杆事件数量 Number of switch events
  };

  /**
   * @brief 键盘与鼠标事件 ID，键盘按键的顺序与 DBUS 按键位一致，编号接在拨杆事件之后。
   *        Keyboard and mouse event IDs. The keyboard keys follow the order of the DBUS
   *        key bits, and the numbers continue after the switch events.
   */
  enum class Key : uint8_t
  {
    KEY_W = static_cast<uint8_t>(SwitchPos::DR16_SW_POS_NUM),  ///< W 键 W key
    KEY_S,          ///< S 键 S key
    KEY_A,          ///< A 键 A key
    KEY_D,          ///< D 键 D key
    KEY_SHIFT,      ///< Shift 键 Shift key
    KEY_CTRL,       ///< Ctrl 键 Ctrl key
    KEY_Q,          ///< Q 键 Q key
    KEY_E,          ///< E 键 E key
    KEY_R,          ///< R 键 R key
    KEY_F,          ///< F 键 F key
    KEY_G,          ///< G 键 G key
    KEY_Z,          ///< Z 键 Z key
    KEY_X,          ///< X 键 X key
    KEY_C,          ///< C 键 C key
    KEY_V,          ///< V 键 V key
    KEY_B,          ///< B 键 B key
    KEY_L_PRESS,    ///< 鼠标左键按下 Left mouse button pressed
    KEY_R_PRESS,    ///< 鼠标右键按下 Right mouse button pressed
    KEY_L_RELEASE,  ///< 鼠标左键释放 Left mouse button released
    KEY_R_RELEASE,  ///< 鼠标右键释放 Right mouse button released
    KEY_NUM,        ///< 事件 ID 数量，也是修饰键事件 ID 的偏移单位
                    ///< Number of event IDs, also the offset unit of modifier IDs
  };

  /**
   * @brief 计算 Shift 组合键的事件 ID。
   *        Compute the event ID of a key pressed together with Shift.
   *
   * @param key 基础按键。
   *            Base key.
   * @return Shift 组合键的事件 ID。
   *         Event ID of the Shift combination.
   */
  constexpr uint32_t ShiftWith(Key key)
  {
    return static_cast<uint8_t>(key) + 1 * static_cast<uint8_t>(Key::KEY_NUM);
  }

  /**
   * @brief 计算 Ctrl 组合键的事件 ID。
   *        Compute the event ID of a key pressed together with Ctrl.
   *
   * @param key 基础按键。
   *            Base key.
   * @return Ctrl 组合键的事件 ID。
   *         Event ID of the Ctrl combination.
   */
  constexpr uint32_t CtrlWith(Key key)
  {
    return static_cast<uint8_t>(key) + 2 * static_cast<uint8_t>(Key::KEY_NUM);
  }

  /**
   * @brief 计算 Shift + Ctrl 组合键的事件 ID。
   *        Compute the event ID of a key pressed together with Shift and Ctrl.
   *
   * @param key 基础按键。
   *            Base key.
   * @return Shift + Ctrl 组合键的事件 ID。
   *         Event ID of the Shift + Ctrl combination.
   */
  constexpr uint32_t ShiftCtrlWith(Key key)
  {
    return static_cast<uint8_t>(key) + 3 * static_cast<uint8_t>(Key::KEY_NUM);
  }

  /**
   * @brief 返回键盘按键在 DBUS 按键位中的掩码。
   *        Return the mask of a keyboard key in the DBUS key bits.
   *
   * @param key 键盘按键。
   *            Keyboard key.
   * @return 按键位掩码；`key` 不小于 `Key::KEY_NUM` 时为 0。
   *         Key bit mask; 0 when `key` is not less than `Key::KEY_NUM`.
   */
  constexpr uint32_t RawValue(Key key)
  {
    if (key >= Key::KEY_NUM)
    {
      return 0;
    }
    return 1 << (static_cast<uint8_t>(key) - static_cast<uint8_t>(Key::KEY_W));
  }

  /**
   * @brief 解析后的 DBUS 帧。
   *        Parsed DBUS frame.
   */
  typedef struct __attribute__((packed))
  {
    uint16_t ch_r_x;  ///< 右摇杆 X 通道 Right stick X channel
    uint16_t ch_r_y;  ///< 右摇杆 Y 通道 Right stick Y channel
    uint16_t ch_l_x;  ///< 左摇杆 X 通道 Left stick X channel
    uint16_t ch_l_y;  ///< 左摇杆 Y 通道 Left stick Y channel
    uint8_t sw_r;     ///< 右拨杆位置 Right switch position
    uint8_t sw_l;     ///< 左拨杆位置 Left switch position
    int16_t x;        ///< 鼠标 X 轴移动 Mouse X movement
    int16_t y;        ///< 鼠标 Y 轴移动 Mouse Y movement
    int16_t z;        ///< 鼠标 Z 轴移动 Mouse Z movement
    uint8_t press_l;  ///< 鼠标左键状态 Left mouse button state
    uint8_t press_r;  ///< 鼠标右键状态 Right mouse button state
    uint16_t key;     ///< 键盘按键位 Keyboard key bits
    uint16_t res;     ///< 拨轮值 Wheel value
  } Data;

  /**
   * @brief DR16 配置参数。
   *        DR16 configuration parameters.
   */
  struct Param
  {
    uint32_t task_stack_depth_uart;  ///< 接收线程栈深
                                     ///< Receive thread stack depth
    LibXR::Thread::Priority thread_priority_uart;  ///< 接收线程优先级
                                                   ///< Receive thread priority
  };

  /**
   * @brief 构造 DR16，配置 UART 并创建接收线程。
   *        Construct DR16, configure the UART and create the receive thread.
   *
   * @param uart 连接 DR16 接收机的 UART，被设置为 100000 bit/s、偶校验、8 数据位、
   *             1 停止位。
   *             UART connected to the DR16 receiver, set to 100000 bit/s, even parity,
   *             8 data bits and 1 stop bit.
   * @param cmd CMD 实例，接收解析后的控制数据。
   *            CMD instance that receives the parsed control data.
   * @param param 配置参数。
   *              Configuration parameters.
   */
  DR16(LibXR::UART& uart, CMD& cmd,
       const Param& param = {.task_stack_depth_uart = 2048,
                             .thread_priority_uart = LibXR::Thread::Priority::MEDIUM})
      : cmd_(&cmd), uart_(std::addressof(uart)), sem_(0), op_(sem_, 4)
  {
    uart_->SetConfig({100000, LibXR::UART::Parity::EVEN, 8, 1});
    /* 创建UART线程 */
    thread_uart_.Create(this, ThreadDr16, "uart_dr16", param.task_stack_depth_uart,
                        param.thread_priority_uart);
  }

  /**
   * @brief 获取 DR16 的事件对象，拨杆、按键和鼠标事件在其上触发。
   *        Get the event object of DR16, on which the switch, key and mouse events are
   *        activated.
   *
   * @return 事件对象的引用。
   *         Reference to the event object.
   */
  LibXR::Event& GetEvent() { return dr16_event_; }

  /**
   * @brief 接收线程函数：每 5 ms 读取并解析一帧 DBUS 数据，超过 100 ms 无有效帧时
   *        判定为离线。
   *        Receive thread function that reads and parses one DBUS frame every 5 ms and
   *        treats DR16 as offline when no valid frame arrives for more than 100 ms.
   *
   * @param dr16 DR16 实例指针。
   *             Pointer to the DR16 instance.
   */
  static void ThreadDr16(DR16* dr16)
  {
    constexpr std::size_t RX_BUFFER_SIZE = 18;
    uint8_t rx_buffer[RX_BUFFER_SIZE] = {0};
    CMD::Data rc_data;

    auto last_time = LibXR::Timebase::GetMilliseconds();
    while (1)
    {
      if (dr16->uart_->Read({rx_buffer, RX_BUFFER_SIZE}, dr16->op_) ==
          LibXR::ErrorCode::OK)
      {
        if (dr16->ParseRC(rx_buffer, rc_data) == LibXR::ErrorCode::OK)
        {
          dr16->last_time_ = LibXR::Timebase::GetMilliseconds();
          dr16->cmd_->FeedRC(CMD::RCInputSource::RC_INPUT_DR16, rc_data);
        }
        else
        {
          LibXR::Memory::FastSet(rx_buffer, 0, RX_BUFFER_SIZE);
        }
      }
      dr16->CheckoutOffline();
      LibXR::Thread::SleepUntil(last_time, 5);
    }
  }

  /**
   * @brief 解析 DBUS 原始数据，生成 CMD 控制数据并触发拨杆、按键和鼠标事件。
   *        Parse raw DBUS data, generate the CMD control data and activate the switch,
   *        key and mouse events.
   *
   * @param raw_data 18 字节的 DBUS 原始数据。
   *                 18 bytes of raw DBUS data.
   * @param output_data 输出的 CMD 控制数据。
   *                    Output CMD control data.
   * @return `OK` 表示解析成功；`PTR_NULL` 表示 `raw_data` 为空；`CHECK_ERR` 表示
   *         摇杆通道超出 364 至 1684 或拨杆值为 0。
   *         `OK` on success; `PTR_NULL` when `raw_data` is null; `CHECK_ERR` when a stick
   *         channel is outside 364 to 1684 or a switch value is 0.
   */
  LibXR::ErrorCode ParseRC(const uint8_t* raw_data, CMD::Data& output_data)
  {
    if (!raw_data)
    {
      return LibXR::ErrorCode::PTR_NULL;
    };

    Data curr_rc{};

    curr_rc.ch_r_x = ((raw_data[0] | raw_data[1] << 8) & 0x07FF);
    curr_rc.ch_r_y = ((raw_data[1] >> 3 | raw_data[2] << 5) & 0x07FF);
    curr_rc.ch_l_x = ((raw_data[2] >> 6 | raw_data[3] << 2 | raw_data[4] << 10) & 0x07FF);
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

    if (curr_rc.ch_l_x < DR16_CH_VALUE_MIN || curr_rc.ch_l_x > DR16_CH_VALUE_MAX ||
        curr_rc.ch_l_y < DR16_CH_VALUE_MIN || curr_rc.ch_l_y > DR16_CH_VALUE_MAX ||
        curr_rc.ch_r_x < DR16_CH_VALUE_MIN || curr_rc.ch_r_x > DR16_CH_VALUE_MAX ||
        curr_rc.ch_r_y < DR16_CH_VALUE_MIN || curr_rc.ch_r_y > DR16_CH_VALUE_MAX)
    {
      return LibXR::ErrorCode::CHECK_ERR;
    }

    if (curr_rc.sw_l == 0 || curr_rc.sw_r == 0)
    {
      return LibXR::ErrorCode::CHECK_ERR;
    }

    output_data = CMD::Data();

    if (curr_rc.sw_l != this->last_data_.sw_l)
    {
      this->dr16_event_.Active(static_cast<uint32_t>(SwitchPos::DR16_SW_L_POS_TOP) +
                               curr_rc.sw_l - 1);
    }
    if (curr_rc.sw_r != this->last_data_.sw_r)
    {
      this->dr16_event_.Active(static_cast<uint32_t>(SwitchPos::DR16_SW_R_POS_TOP) +
                               curr_rc.sw_r - 1);
    }

    uint32_t modifier_offset = 0;

    if (curr_rc.key & RawValue(Key::KEY_SHIFT))
    {
      modifier_offset += static_cast<uint32_t>(Key::KEY_NUM);
    }
    if (curr_rc.key & RawValue(Key::KEY_CTRL))
    {
      modifier_offset += 2 * static_cast<uint32_t>(Key::KEY_NUM);
    }

    for (int i = 0; i < 16; i++)
    {
      if ((curr_rc.key & (1 << i)) && !(this->last_data_.key & (1 << i)))
      {
        this->dr16_event_.Active(static_cast<uint32_t>(Key::KEY_W) + i + modifier_offset);
      }
    }

    constexpr float FULL_RANGE =
        static_cast<float>(DR16_CH_VALUE_MAX - DR16_CH_VALUE_MIN);
    constexpr float INV_FULL_RANGE = 1.0f / FULL_RANGE;
    constexpr float MOUSE_SCALER = 20.0f / 32768.0f;

    if (curr_rc.press_l && !this->last_data_.press_l)
    {
      this->dr16_event_.Active(static_cast<uint32_t>(Key::KEY_L_PRESS));
    }
    if (!curr_rc.press_l && this->last_data_.press_l)
    {
      this->dr16_event_.Active(static_cast<uint32_t>(Key::KEY_L_RELEASE));
    }
    if (curr_rc.press_r && !this->last_data_.press_r)
    {
      this->dr16_event_.Active(static_cast<uint32_t>(Key::KEY_R_PRESS));
    }
    if (!curr_rc.press_r && this->last_data_.press_r)
    {
      this->dr16_event_.Active(static_cast<uint32_t>(Key::KEY_R_RELEASE));
    }

    output_data.chassis.x =
        2 * (static_cast<float>(curr_rc.ch_l_x) - DR16_CH_VALUE_MID) * INV_FULL_RANGE;
    output_data.chassis.y =
        2 * (static_cast<float>(curr_rc.ch_l_y) - DR16_CH_VALUE_MID) * INV_FULL_RANGE;
    output_data.chassis.z =
        -2 * (static_cast<float>(curr_rc.ch_r_x) - DR16_CH_VALUE_MID) * INV_FULL_RANGE;

    output_data.gimbal.yaw =
        -2 * (static_cast<float>(curr_rc.ch_r_x) - DR16_CH_VALUE_MID) * INV_FULL_RANGE;
    output_data.gimbal.pit =
        2 * (static_cast<float>(curr_rc.ch_r_y) - DR16_CH_VALUE_MID) * INV_FULL_RANGE;

    if (curr_rc.key & RawValue(Key::KEY_A))
    {
      output_data.chassis.x -= 1.0f;
    }
    if (curr_rc.key & RawValue(Key::KEY_D))
    {
      output_data.chassis.x += 1.0f;
    }
    if (curr_rc.key & RawValue(Key::KEY_S))
    {
      output_data.chassis.y -= 1.0f;
    }
    if (curr_rc.key & RawValue(Key::KEY_W))
    {
      output_data.chassis.y += 1.0f;
    }

    output_data.chassis.self_define = CMD::ChasStat::NONE;

    output_data.gimbal.pit += static_cast<float>(curr_rc.y) * MOUSE_SCALER;
    output_data.gimbal.yaw += -static_cast<float>(curr_rc.x) * MOUSE_SCALER;

    if (curr_rc.key & RawValue(Key::KEY_SHIFT) or curr_rc.res == DR16_CH_VALUE_MAX)
    {
      output_data.chassis.self_define = CMD::ChasStat::BOOST;
    }

    if (curr_rc.key & RawValue(Key::KEY_C) or curr_rc.res == DR16_CH_VALUE_MIN)
    {
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

  /**
   * @brief 向 CMD 提交离线数据：控制量全零，底盘与云台在线标志为 false。
   *        Submit offline data to CMD: all control values zero and the chassis and gimbal
   *        online flags false.
   */
  void Offline()
  {
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
   * @brief 调试用的非紧凑数据视图，字段与 `Data` 相同。
   *        Unpacked data view for debugging, with the same fields as `Data`.
   */
  struct DataView
  {
    uint16_t ch_r_x;  ///< 右摇杆 X 通道 Right stick X channel
    uint16_t ch_r_y;  ///< 右摇杆 Y 通道 Right stick Y channel
    uint16_t ch_l_x;  ///< 左摇杆 X 通道 Left stick X channel
    uint16_t ch_l_y;  ///< 左摇杆 Y 通道 Left stick Y channel
    uint8_t sw_r;     ///< 右拨杆位置 Right switch position
    uint8_t sw_l;     ///< 左拨杆位置 Left switch position
    int16_t x;        ///< 鼠标 X 轴移动 Mouse X movement
    int16_t y;        ///< 鼠标 Y 轴移动 Mouse Y movement
    int16_t z;        ///< 鼠标 Z 轴移动 Mouse Z movement
    uint8_t press_l;  ///< 鼠标左键状态 Left mouse button state
    uint8_t press_r;  ///< 鼠标右键状态 Right mouse button state
    uint16_t key;     ///< 键盘按键位 Keyboard key bits
    uint16_t res;     ///< 拨轮值 Wheel value
  };

  /**
   * @brief 将紧凑的 `Data` 复制到 `DataView`。
   *        Copy a packed `Data` into a `DataView`.
   *
   * @param data_view 输出的数据视图。
   *                  Output data view.
   * @param data 输入的数据帧。
   *             Input data frame.
   */
  void DataviewToData(DataView& data_view, Data& data)
  {
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

  void CheckoutOffline()
  {
    auto current_time = LibXR::Timebase::GetMilliseconds();
    if ((current_time - last_time_).ToMillisecond() > 100)
    {
      Offline();
    }
  }
};
