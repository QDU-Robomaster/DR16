#pragma once

// clang-format off
/* === MODULE MANIFEST ===
module_name: DR16
module_description: Receiver parsing
constructor_args:
  - task_stack_depth_uart: 2048
required_hardware: dr16 dma uart
=== END MANIFEST === */
// clang-format on

#include "app_framework.hpp"
#include "uart.hpp"

#define DR16_CH_VALUE_MIN (364u)
#define DR16_CH_VALUE_MID (1024u)
#define DR16_CH_VALUE_MAX (1684u)

class DR16 : public LibXR::Application {
 public:
  enum class ControlSource : uint8_t {
    DR16_CTRL_SOURCE_SW = 0x00,
    DR16_CTRL_SOURCE_MOUSE = 0x01,
  };

  enum class SwitchPos : uint8_t {
    DR16_SW_L_POS_TOP = 0x00,
    DR16_SW_L_POS_BOT = 0x01,
    DR16_SW_L_POS_MID = 0x02,
    DR16_SW_R_POS_TOP = 0x03,
    DR16_SW_R_POS_BOT = 0x04,
    DR16_SW_R_POS_MID = 0x05,
    DR16_SW_POS_NUM
  };

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

  constexpr uint32_t ShiftWith(Key key) {
    return static_cast<uint8_t>(key) + 1 * static_cast<uint8_t>(Key::KEY_NUM);
  }
  constexpr uint32_t CtrlWith(Key key) {
    return static_cast<uint8_t>(key) + 2 * static_cast<uint8_t>(Key::KEY_NUM);
  }
  constexpr uint32_t ShiftCtrlWith(Key key) {
    return static_cast<uint8_t>(key) + 3 * static_cast<uint8_t>(Key::KEY_NUM);
  }

  typedef struct __attribute__((packed)) {
    uint16_t ch_r_x : 11;
    uint16_t ch_r_y : 11;
    uint16_t ch_l_x : 11;
    uint16_t ch_l_y : 11;
    uint8_t sw_r : 2;
    uint8_t sw_l : 2;
    int16_t x;
    int16_t y;
    int16_t z;
    uint8_t press_l;
    uint8_t press_r;
    uint16_t key;
    uint16_t res;
  } Data;

  // 用于直接查看数据的普通结构体（不使用位域）
  struct DataView {
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
  };

  DR16(LibXR::HardwareContainer &hw, LibXR::ApplicationManager &app,
       uint32_t task_stack_depth_uart)
      : uart_(hw.template Find<LibXR::UART>("uart_dr16")), sem(0), op(sem) {
    uart_->SetConfig({100000, LibXR::UART::Parity::EVEN, 8, 1});

    cmd_tp_ = LibXR::Topic::CreateTopic<Data>("dr16_cmd", nullptr, true);

    thread_uart_.Create(this, Thread_Dr16, "uart_dr16", task_stack_depth_uart,
                        LibXR::Thread::Priority::HIGH);
    app.Register(*this);
  }

  void OnMonitor() override {}

  static void Thread_Dr16(DR16 *dr16) {
    dr16->uart_->read_port_->Reset();

    while (true) {
      dr16->uart_->Read(dr16->data_, dr16->op);
      if (dr16->DataCorrupted()) {
        dr16->uart_->read_port_->Reset();
        LibXR::Thread::Sleep(3);
      } else {
        dr16->DataviewToData(dr16->data_view_, dr16->data_);
        dr16->cmd_tp_.Publish(dr16->data_);
      }
    }
  }

  bool DataCorrupted() {
    if ((data_.ch_r_x < DR16_CH_VALUE_MIN) ||
        (data_.ch_r_x > DR16_CH_VALUE_MAX)) {
      return true;
    }
    if ((data_.ch_r_y < DR16_CH_VALUE_MIN) ||
        (data_.ch_r_y > DR16_CH_VALUE_MAX)) {
      return true;
    }
    if ((data_.ch_l_x < DR16_CH_VALUE_MIN) ||
        (data_.ch_l_x > DR16_CH_VALUE_MAX)) {
      return true;
    }
    if ((data_.ch_l_y < DR16_CH_VALUE_MIN) ||
        (data_.ch_l_y > DR16_CH_VALUE_MAX)) {
      return true;
    }
    if (data_.sw_l == 0) {
      return true;
    }

    if (data_.sw_r == 0) {
      return true;
    }

    return false;
  }
void DataviewToData(DataView &data_view, Data &data) {
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

 private:
  Data data_;
  DataView data_view_;

  LibXR::UART *uart_;
  LibXR::Thread thread_uart_;
  LibXR::Semaphore sem;
  LibXR::ReadOperation op;
  LibXR::Topic cmd_tp_;
};
