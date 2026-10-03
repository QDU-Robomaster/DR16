# DR16

DR16 遥控接收机解析模块：从 UART 接收 DBUS 数据并转换为 CMD 控制数据 / Module that parses DR16 receiver DBUS data from a UART and passes it to CMD as control data

## 1. 模块作用 / Purpose

构造时，DR16 把 UART 配置为 100000 bit/s、偶校验、8 数据位、1 停止位，并创建线程 `uart_dr16`（栈深与优先级见 `Param`）。线程每 5 ms 读取一帧 18 字节的 DBUS 数据，读超时 4 ms。四个摇杆通道超出 364 至 1684，或任一拨杆值为 0 时，该帧被丢弃；有效帧转换为 `CMD::Data`，通过 `cmd.FeedRC(CMD::RCInputSource::RC_INPUT_DR16, ...)` 提交给 CMD。超过 100 ms 没有有效帧时判定为离线，此后每个周期向 CMD 提交控制量全零、`chassis_online` 与 `gimbal_online` 为 `false` 的数据。

转换到 `CMD::Data` 的映射如下：

- 底盘：`x` 为左摇杆 X，`y` 为左摇杆 Y，`z` 为右摇杆 X 取反，均归一化到 [-1, 1]；键盘 A / D 在 `x` 上减 / 加 1，S / W 在 `y` 上减 / 加 1；结果限幅到 [-1, 1]。
- 云台：`yaw` 为右摇杆 X 取反再减去鼠标 X × 20/32768，`pit` 为右摇杆 Y 加上鼠标 Y × 20/32768。
- 底盘模式：Shift 按下或拨轮（`res`）为最大值 1684 时为 `BOOST`；C 按下或拨轮为最小值 364 时为 `STRETCH`，`STRETCH` 优先。
- 开火：拨轮为最小值 364 或鼠标左键按下。

`GetEvent()` 返回 DR16 的 `LibXR::Event`，EventBinder 等模块用它绑定下列事件 ID：

- 拨杆位置变化：左拨杆 `DR16_SW_L_POS_TOP`、`DR16_SW_L_POS_BOT`、`DR16_SW_L_POS_MID`（0 至 2），右拨杆 `DR16_SW_R_POS_TOP`、`DR16_SW_R_POS_BOT`、`DR16_SW_R_POS_MID`（3 至 5）。
- 键盘按键按下（上升沿）：`Key::KEY_W` 至 `Key::KEY_B`。同时按住 Shift、Ctrl、Shift + Ctrl 时，事件 ID 分别加上 1、2、3 倍 `Key::KEY_NUM`，可用 `ShiftWith()`、`CtrlWith()`、`ShiftCtrlWith()` 计算。
- 鼠标：`Key::KEY_L_PRESS`、`Key::KEY_R_PRESS`、`Key::KEY_L_RELEASE`、`Key::KEY_R_RELEASE`。

Upon construction, DR16 configures the UART to 100000 bit/s, even parity, 8 data bits and 1 stop bit, and creates the thread `uart_dr16` (stack depth and priority from `Param`). Every 5 ms the thread reads one 18-byte DBUS frame with a 4 ms read timeout. A frame is discarded when any of the four stick channels is outside 364 to 1684 or either switch value is 0. A valid frame is converted to `CMD::Data` and submitted to CMD through `cmd.FeedRC(CMD::RCInputSource::RC_INPUT_DR16, ...)`. When no valid frame has arrived for more than 100 ms, DR16 is considered offline, and in every following cycle it submits data to CMD with all control values zero and `chassis_online` and `gimbal_online` set to `false`.

The mapping to `CMD::Data` is:

- Chassis: `x` is the left stick X, `y` is the left stick Y and `z` is the negated right stick X, all normalized to [-1, 1]; keyboard A / D subtract / add 1 on `x` and S / W subtract / add 1 on `y`; the results are clamped to [-1, 1].
- Gimbal: `yaw` is the negated right stick X minus the mouse X × 20/32768, and `pit` is the right stick Y plus the mouse Y × 20/32768.
- Chassis mode: `BOOST` when Shift is pressed or the wheel (`res`) is at its maximum 1684; `STRETCH` when C is pressed or the wheel is at its minimum 364, with `STRETCH` taking priority.
- Fire: the wheel is at its minimum 364 or the left mouse button is pressed.

`GetEvent()` returns the `LibXR::Event` of DR16, to which Modules such as EventBinder bind the following event IDs:

- Switch position changes: left switch `DR16_SW_L_POS_TOP`, `DR16_SW_L_POS_BOT`, `DR16_SW_L_POS_MID` (0 to 2), right switch `DR16_SW_R_POS_TOP`, `DR16_SW_R_POS_BOT`, `DR16_SW_R_POS_MID` (3 to 5).
- Keyboard key presses (rising edge): `Key::KEY_W` to `Key::KEY_B`. While Shift, Ctrl or Shift + Ctrl is held, the event ID is increased by 1, 2 or 3 times `Key::KEY_NUM`, which `ShiftWith()`, `CtrlWith()` and `ShiftCtrlWith()` compute.
- Mouse: `Key::KEY_L_PRESS`, `Key::KEY_R_PRESS`, `Key::KEY_L_RELEASE`, `Key::KEY_R_RELEASE`.

## 2. 构造接口 / Constructor

```cpp
DR16(LibXR::UART& uart,
     CMD& cmd,
     const Param& param = {.task_stack_depth_uart = 2048,
                           .thread_priority_uart = LibXR::Thread::Priority::MEDIUM});
```

依赖：

- `uart`：连接 DR16 接收机的 `LibXR::UART`，取自 BSP 的硬件注册（`XR_REGISTER`）。
- `cmd`：`CMD` 实例，接收解析后的控制数据。

配置参数（`Param`）：

- `task_stack_depth_uart`：接收线程栈深，默认 2048。
- `thread_priority_uart`：接收线程优先级，默认 `LibXR::Thread::Priority::MEDIUM`。

Dependencies:

- `uart`: the `LibXR::UART` connected to the DR16 receiver, taken from the BSP's Registration (`XR_REGISTER`).
- `cmd`: the `CMD` instance that receives the parsed control data.

Configuration parameters (`Param`):

- `task_stack_depth_uart`: receive thread stack depth, default 2048.
- `thread_priority_uart`: receive thread priority, default `LibXR::Thread::Priority::MEDIUM`.

## 3. Topic

无 / None

## 4. 配置示例 / Configuration Example

`xrobot instance add QDU-Robomaster/DR16` 写入的实例，`uart` 填写为 BSP 中注册的 UART 名称，`cmd` 填写为 `QDU-Robomaster/CMD` 实例的 id，该实例在 `modules:` 中排在 DR16 之前：

An instance written by `xrobot instance add QDU-Robomaster/DR16`, with `uart` set to a UART name registered by the BSP and `cmd` set to the id of the `QDU-Robomaster/CMD` instance, which is listed before DR16 in `modules:`:

```yaml
modules:
  - module: QDU-Robomaster/DR16
    id: dr16
    args:
      - uart: usart3
      - cmd: cmd
      - param:
          task_stack_depth_uart: 1536
          thread_priority_uart: LibXR::Thread::Priority::MEDIUM
```

## 5. 依赖与硬件 / Dependencies and Hardware

依赖：

- `QDU-Robomaster/CMD`：接收解析后的控制数据（`CMD::FeedRC`）。
- LibXR。

硬件：连接 DR16 接收机 DBUS 输出的 UART，并通过 `XR_REGISTER` 注册；波特率与校验由 DR16 在构造时设置。

Dependencies:

- `QDU-Robomaster/CMD`: receives the parsed control data (`CMD::FeedRC`).
- LibXR.

Hardware: a UART connected to the DBUS output of the DR16 receiver and registered with `XR_REGISTER`; DR16 sets the baud rate and parity at construction.
