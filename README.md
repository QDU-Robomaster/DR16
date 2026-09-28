# DR16

DR16 遥控解析模块：从 UART 接收 DBUS 数据，转换为 `CMD::Data` 喂给 CMD，
并把拨杆、键盘和鼠标按键变化作为事件发出。

## 行为

- 构造时把 UART 设为 100000 bit/s、偶校验、8 数据位、1 停止位，并创建线程
  `uart_dr16`（栈深与优先级见 `Param`）。
- 线程每 5 ms 读取一帧 18 字节 DBUS 数据（读超时 4 ms）。四个摇杆通道超出 364-1684
  或拨杆值为 0 时丢弃该帧。有效帧通过 `cmd.FeedRC(CMD::RCInputSource::RC_INPUT_DR16, ...)`
  喂给 CMD。
- 超过 100 ms 没有有效帧时判定离线：每个周期向 CMD 喂入控制量全零、
  `chassis_online` / `gimbal_online` 为 `false` 的数据。
- 映射到 `CMD::Data`：
  - 底盘：`x` = 左摇杆 X，`y` = 左摇杆 Y，`z` = -右摇杆 X，归一化到 [-1, 1]；
    键盘 A/D、S/W 在 `x`、`y` 上各加减 1；结果限幅到 [-1, 1]。
  - 云台：`yaw` = -右摇杆 X − 鼠标 X × 20/32768，`pit` = 右摇杆 Y + 鼠标 Y × 20/32768。
  - 底盘模式：Shift 或拨轮（`res`）到最大值 1684 时为 `BOOST`；C 或拨轮到最小值 364 时为
    `STRETCH`（优先）。
  - 开火：拨轮到最小值 364 或鼠标左键按下。

## 事件

`GetEvent()` 返回 DR16 的 `LibXR::Event`，EventBinder 等模块用它绑定下列事件 ID：

- 拨杆位置变化：左拨杆 `DR16_SW_L_POS_TOP` / `BOT` / `MID`（0-2），右拨杆
  `DR16_SW_R_POS_TOP` / `BOT` / `MID`（3-5）。
- 键盘按下（上升沿）：`Key::KEY_W` ... `KEY_B`；同时按住 Shift / Ctrl / Shift+Ctrl 时，
  事件 ID 分别加上 1 / 2 / 3 倍 `KEY_NUM`，可用 `ShiftWith()`、`CtrlWith()`、
  `ShiftCtrlWith()` 计算。
- 鼠标：`KEY_L_PRESS`、`KEY_R_PRESS`、`KEY_L_RELEASE`、`KEY_R_RELEASE`。

## 依赖

- `QDU-Robomaster/CMD`：接收解析后的控制数据（`CMD::FeedRC`）。

## 构造接口

```cpp
DR16(LibXR::UART& uart,
     CMD& cmd,
     const Param& param = {.task_stack_depth_uart = 2048,
                           .thread_priority_uart = LibXR::Thread::Priority::MEDIUM});
```

依赖：

- `uart`：连接 DR16 接收机的 `LibXR::UART`（模块会重新设置波特率与校验）。
- `cmd`：CMD 模块实例，填写 CMD 的实例 id。

配置（`Param` 字段）：

- `task_stack_depth_uart`：接收线程栈深，默认 2048。
- `thread_priority_uart`：接收线程优先级，默认 `LibXR::Thread::Priority::MEDIUM`。

## 使用

```sh
xrobot module add QDU-Robomaster/DR16
xrobot setup
xrobot instance add QDU-Robomaster/DR16
```

`xrobot instance add` 在 `User/xrobot.yaml` 中写入一个实例，依赖项留空，默认值按源码写出；
把 `uart` 填为 BSP 中用 `XR_REGISTER` 注册的 UART 对象名，把 `cmd` 填为前面列出的
`QDU-Robomaster/CMD` 实例的 id：

```yaml
modules:
  - module: QDU-Robomaster/CMD
    id: cmd
    args:
      - mode: CMD::Mode::CMD_OP_CTRL
      - chassis_cmd_topic_name: '"chassis_cmd"'
      - gimbal_cmd_topic_name: '"gimbal_cmd"'
      - launcher_cmd_topic_name: '"launcher_cmd"'
  - module: QDU-Robomaster/DR16
    id: dr16_0
    args:
      - uart: uart_dr16
      - cmd: cmd
      - param:
          task_stack_depth_uart: '2048'
          thread_priority_uart: LibXR::Thread::Priority::MEDIUM
```

BSP 侧：

```cpp
XR_REGISTER(uart_dr16, LibXR::UART);
```

`cmd` 是 `QDU-Robomaster/CMD` 的实例 id，该实例必须在 `modules:` 中排在 DR16 前面。

填好后再次运行 `xrobot setup`，生成 `User/xrobot_main.hpp`。

`xrobot module show .`（在本仓库中）或 `xrobot module show Modules/QDU-Robomaster/DR16`
（在 BSP 中）打印 manifest 和当前的构造函数。
