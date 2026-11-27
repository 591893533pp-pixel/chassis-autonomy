# elmue_can

> libusb 用户态 CANable 2.5 (ElmüSoft Candlelight 2.5) 驱动，用于 Robstride RS00/RS03/RS04/RS05/RS10/EL05 电机。

## 特点

- 完全用户态 (libusb)，无需内核 `gs_usb` 驱动，绕开 SocketCAN 写队列阻塞
- 单适配器 6900 Hz × 1 h 通信 0 丢包（已验证 24,839,921 帧）
- 同进程可同时打开多张 CANable 2.5（按 USB 序列号区分），适合双臂部署
- 同时输出 C++ 共享库 `libelmue_can.so` 和 Python 扩展模块 `elmue_can._ext`

## 构建

```bash
# 一次性依赖安装
sudo bash ../scripts/install_elmue_deps.sh

# 构建
cmake -B build && cmake --build build -j
# 产物:
#   build/libelmue_can.so                       — C++ 共享库
#   build/elmue_can/_ext.cpython-*.so           — Python 扩展
#   build/elmue_can/__init__.py                 — Python 包入口
#   build/rs00_comm_test                        — 多电机通信压测工具
```

构建选项：

| 选项 | 默认 | 说明 |
|------|------|------|
| `ELMUE_BUILD_PYBIND` | `ON` | 是否构建 pybind11 Python 扩展 |
| `ELMUE_BUILD_TESTS` | `ON` | 是否构建 `rs00_comm_test` 压测程序 |

## 使用前置：解绑内核 gs_usb

CANable 2.5 出厂被 `gs_usb` 内核驱动接管（会以 SocketCAN `canX` 接口形式注册）。
本库需要 libusb 独占设备，因此必须先解绑：

```bash
sudo ../scripts/unbind_gs_usb.sh list      # 查看当前状态
sudo ../scripts/unbind_gs_usb.sh unbind    # 解绑 gs_usb
sudo ../scripts/unbind_gs_usb.sh rebind    # 恢复 SocketCAN
```

## C++ API 概览

```cpp
#include "elmue_can/robstride_elmue.h"
using namespace elmue_can;

RobstrideElmueDriver drv(/*host_can_id=*/0xFD);
drv.init("208D337938455002", /*bitrate=*/1'000'000, /*dbitrate=*/5'000'000);

drv.setMotorType(1, MotorType::RS04);
drv.enableMotor(1);
drv.sendMotionControl(1, MotorType::RS04, /*pos=*/0.0, /*vel=*/0.0,
                      /*kp=*/80.0, /*kd=*/4.0, /*tau=*/0.0);
auto fb = drv.getMotorFeedback(1);
printf("position=%.3f velocity=%.3f\n", fb.position, fb.velocity);
```

## Python API 概览

```python
import sys; sys.path.insert(0, "elmue_can/build")
from elmue_can import RobstrideElmueDriver, MotorType

drv = RobstrideElmueDriver(host_can_id=0xFD)
drv.init("208D337938455002")
drv.set_motor_type(1, MotorType.RS04)
drv.enable_motor(1)
drv.send_motion_control(1, MotorType.RS04, 0.0, 0.0, 80.0, 4.0, 0.0)
fb = drv.get_motor_feedback(1)
print(fb.position, fb.velocity)
```

## 多电机压测

```bash
# 单电机 6900 Hz × 30 s
sudo ./build/rs00_comm_test --serial 208D337938455002 --motor-ids 127 --hz 6900

# 整臂 7 电机 1 kHz × 60 s
sudo ./build/rs00_comm_test --serial <SN> --motor-ids 1-7 --hz 1000 --duration 60
```

完整 CLI 参考：`./build/rs00_comm_test --help`

## 与 POKE 上层栈集成

- **Python SDK** ([`poke_sdk/`](../poke_sdk/))：`POKEInterface(backend="elmue", elmue_serial="<SN>")`，详见 [`poke_sdk/poke/elmue_can_driver.py`](../poke_sdk/poke/elmue_can_driver.py)
- **ROS2** ([`poke_ros/poke_hardware/`](../poke_ros/poke_hardware/))：在 xacro `<hardware>` 中设置 `transport=elmue` + `serial_number=<SN>`，详见 [HARDWARE_INTERFACE.md](../poke_ros/poke_hardware/HARDWARE_INTERFACE.md)
