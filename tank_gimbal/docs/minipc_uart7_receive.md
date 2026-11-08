# Mini PC 接收云台 UART7 遥控器数据

本文对应当前 `tank_gimbal` 工作区源码中的 `GimbalToArm` 协议。云台解析遥控器后，通过 UART7 向 Mini PC 周期发送拨杆、键盘和处理后的状态。Mini PC 不需要解码原始 18 字节 DBUS 数据。

## 1. 接线和串口参数

| 云台 STM32 信号 | Mini PC 侧 | 说明 |
| --- | --- | --- |
| `PE8 / UART7_TX` | USB 转串口模块的 `RX`，或 Mini PC 的 3.3 V TTL UART `RX` | 数据方向：云台 → Mini PC |
| `GND` | USB 转串口模块或 Mini PC 的 `GND` | 两侧必须共地 |

这是单向发送协议，当前接收示例不需要连接 Mini PC 的 TX。`PE7` 是云台 `UART7_RX`，不是这里的发送脚。上表是 MCU 引脚功能；如果使用板载接插件，接线前需根据实际原理图确认 `PE8` 接到了哪个引脚、是否经过电平转换，不能仅凭 MCU 管脚名确定接口位置。

MCU 侧是 3.3 V 逻辑 UART。使用 USB 转串口时请选支持 **921600 波特率、3.3 V TTL 电平**的模块；不要将 RS-232 电平直接接到 STM32 引脚。此接口也不是底盘使用的 RS485 总线。

| 参数 | 值 |
| --- | --- |
| 波特率 | `921600` |
| 数据位 | 8 位 |
| 校验 | 无 |
| 停止位 | 1 位，即 `8N1` |
| 流控 | 无 |
| 字节顺序 | 多字节整数均为小端，低字节在前 |
| 发送周期 | 云台任务每次循环尝试发送，延时为 2 个系统 tick（当前约 2 ms）；调度和 DMA 忙会造成间隔变化 |

**串口号由 Mini PC 系统分配**：Linux 常见 `/dev/ttyUSB0` 或 `/dev/ttyACM0`，Windows 常见 `COM3` 等，具体以实际设备为准。USB 转串口模块的接收能力和实际接线需要实测。

## 2. 固定 12 字节数据帧

| 字节偏移 | 长度 | 字段 | 说明 |
| --- | --- | --- | --- |
| 0–1 | 2 | 帧头 | 固定 `25 44`，按这个顺序发送 |
| 2 | 1 | `s1` | 遥控器 S1 拨杆 |
| 3 | 1 | `s2` | 遥控器 S2 拨杆 |
| 4–5 | 2 | `keyboard` | 原始键盘位掩码，`uint16` 小端 |
| 6–7 | 2 | `f_count` | F 键 4 秒统计结果，`uint16` 小端；非零结果仅发送一帧 |
| 8 | 1 | `g_state` | G 键翻转状态，`0` 或 `1` |
| 9 | 1 | `remote_online` | `0`：遥控器离线；`1`：遥控器在线 |
| 10–11 | 2 | `crc16` | CRC16/MCRF4XX，低字节在前，覆盖字节 0–9 |

在线时，拨杆值为：`1 = 上`、`3 = 中`、`2 = 下`。双中即 `s1 == 3 && s2 == 3`。离线帧中，字节 2–9 均为 `0`；其中拨杆值 `0` 表示无有效拨杆数据。

`keyboard` 的每一位表示对应按键的当前原始状态：

| 位 | 按键 | 掩码 | 位 | 按键 | 掩码 |
| --- | --- | --- | --- | --- | --- |
| 0 | W | `0x0001` | 8 | R | `0x0100` |
| 1 | S | `0x0002` | 9 | F | `0x0200` |
| 2 | A | `0x0004` | 10 | G | `0x0400` |
| 3 | D | `0x0008` | 11 | Z | `0x0800` |
| 4 | Shift | `0x0010` | 12 | X | `0x1000` |
| 5 | Ctrl | `0x0020` | 13 | C | `0x2000` |
| 6 | Q | `0x0040` | 14 | V | `0x4000` |
| 7 | E | `0x0080` | 15 | B | `0x8000` |

判断 F 原始电平的例子：`(keyboard & 0x0200) != 0`。键盘掩码是原始电平，并非 F、G 消抖后的事件；机械臂若只需要云台处理后的结果，应使用 `f_count` 和 `g_state`。

## 3. F、G 和离线行为

- **F**：只在双中处理，消抖时间 30 ms。第一次确认按下时开始 4 秒计数窗口；窗口内每次确认的新按下计一次。到期后非零结果只放在一帧中，随后恢复为 0。UART7 DMA 忙或启动失败时，云台会保留待发结果，直到某帧 DMA 成功启动；这不代表 Mini PC 已收到。
- **G**：只在双中处理，消抖时间 30 ms。每次确认的新按下使状态在 0、1 之间翻转；长按不重复翻转。状态随在线帧持续发送。
- **进入双中时已按住 F/G**：需要先松开，再按下，才会产生新的计数或翻转。
- **退出双中或遥控器离线**：取消 F 统计并清零，G 状态清零。遥控器超过约 100 ms 没有更新后，云台判定离线；上电后未收到过完整帧也发送离线状态。当前遥控器接收入口按 18 字节长度接收，不应将在线标志理解为对每个上游字节都进行了 CRC 校验。
- **遥控器离线与串口断链不同**：收到 CRC 正确且 `remote_online == 0` 的帧，说明两板通信仍在工作，但遥控器离线。若持续收不到 CRC 正确的完整帧，则可能是云台断电、接线断开、串口参数错误或接收程序异常；Mini PC 必须自行设置接收超时并进入安全状态。

F 结果只有一帧：**接收程序必须处理读到的每一帧**，不能只取最后一帧，否则可能漏掉非零 `f_count`。当前协议没有序号、确认或重传；如果该帧在串口链路上丢失或校验失败，Mini PC 无法从后续帧恢复这个事件。

## 4. CRC 和测试帧

CRC16/MCRF4XX 参数：初值 `0xFFFF`，反射多项式 `0x8408`，无最终异或；只计算帧中前 10 字节，CRC 低字节先发送。标准字符串 `123456789` 的计算结果为 `0x6F91`。

下面两帧可用于接收程序离线验证，数值均为十六进制：

```text
# 遥控器离线：字节 2–9 全 0，CRC = 0x750F
25 44 00 00 00 00 00 00 00 00 0F 75

# 在线、双中、F/G 原始键位按下、F 结果 3、G 状态 1，CRC = 0x41B5
25 44 03 03 00 06 03 00 01 01 B5 41
```

## 5. Python 接收示例

安装 Python 和 `pyserial`，然后查看实际串口名：

```bash
python -m pip install pyserial
python -m serial.tools.list_ports -v
```

Linux 上若因串口权限被拒绝，需要按系统配置为当前用户授予对应设备的访问权限。以下示例将文件保存为 `receive_gimbal.py`：

```python
import argparse
import time

HEADER = b"\x25\x44"
FRAME_SIZE = 12


def crc16_mcrf4xx(data):
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc >> 1) ^ 0x8408) if (crc & 1) else (crc >> 1)
    return crc & 0xFFFF


class FrameParser:
    def __init__(self):
        self.buffer = bytearray()

    def feed(self, data):
        """接收任意长度字节，返回本次解析出的全部有效帧。"""
        self.buffer.extend(data)
        frames = []
        while True:
            start = self.buffer.find(HEADER)
            if start < 0:
                # 保留可能作为下次帧头开头的单个 0x25。
                self.buffer[:] = b"\x25" if self.buffer[-1:] == b"\x25" else b""
                break
            if start:
                del self.buffer[:start]
            if len(self.buffer) < FRAME_SIZE:
                break

            frame = self.buffer[:FRAME_SIZE]
            received_crc = int.from_bytes(frame[10:12], "little")
            online = frame[9]
            g_state = frame[8]
            switches_valid = frame[2] in (1, 2, 3) and frame[3] in (1, 2, 3)
            if (crc16_mcrf4xx(frame[:10]) != received_crc
                    or online not in (0, 1)
                    or g_state not in (0, 1)
                    or (online == 1 and not switches_valid)):
                # 校验失败时逐字节重新找帧头，不丢弃可能重叠的帧。
                del self.buffer[0]
                continue

            frames.append({
                "s1": frame[2],
                "s2": frame[3],
                "keyboard": int.from_bytes(frame[4:6], "little"),
                "f_count": int.from_bytes(frame[6:8], "little"),
                "g_state": g_state,
                "remote_online": bool(online),
            })
            del self.buffer[:FRAME_SIZE]
        return frames


def main():
    import serial  # python -m pip install pyserial

    parser = argparse.ArgumentParser()
    parser.add_argument("port", help="例如 /dev/ttyUSB0 或 COM3")
    parser.add_argument("--timeout", type=float, default=0.1,
                        help="有效帧超时秒数，默认 0.1；按实际调度和安全要求调整")
    args = parser.parse_args()
    if args.timeout <= 0:
        parser.error("--timeout 必须大于 0")

    decoder = FrameParser()
    last_valid = None
    link_online = False
    last_status = None
    with serial.Serial(args.port, 921600, bytesize=8, parity="N", stopbits=1,
                       timeout=0.02, xonxoff=False, rtscts=False, dsrdtr=False) as port:
        print(f"接收 {port.port}，等待有效帧……")
        while True:
            data = port.read(256)
            for frame in decoder.feed(data):
                last_valid = time.monotonic()
                if not link_online:
                    link_online = True
                    print("云台串口链路：收到有效帧")

                if not frame["remote_online"]:
                    if last_status != "remote_offline":
                        print("遥控器离线：机械臂应进入安全状态")
                        last_status = "remote_offline"
                    # 在这里执行或维持机械臂的安全状态。
                    continue

                status = (frame["s1"], frame["s2"], frame["keyboard"], frame["g_state"])
                if status != last_status:
                    s1, s2, keyboard, g_state = status
                    print(f"S1={s1} S2={s2} keyboard=0x{keyboard:04X} G={g_state}")
                    last_status = status
                if frame["f_count"]:
                    # 每个非零结果只出现一帧；在这里逐帧处理，不要只保留最后一帧。
                    print(f"F 统计结果：{frame['f_count']} 次")

            if last_valid is not None and time.monotonic() - last_valid >= args.timeout:
                if link_online:
                    link_online = False
                    last_status = None
                    print("云台串口链路超时：机械臂应进入安全状态")
                    # 在这里执行或维持机械臂的安全状态。


if __name__ == "__main__":
    main()
```

运行示例：

```bash
# Linux：替换为实际串口
python receive_gimbal.py /dev/ttyUSB0

# Windows：替换为实际串口
python receive_gimbal.py COM3
```

示例中的 `--timeout 0.1` 是 **Mini PC 的板间有效帧超时**，不是云台判断遥控器离线的时间。上电后一帧都没收到时，示例停留在“等待有效帧”，实际机械臂程序应默认保持安全状态；收到离线帧或有效帧超时也应保持安全状态。示例只打印并标出控制接入位置，不会实际控制机械臂。

## 6. 云台侧源码对应位置

- [协议声明与字段](../communication_between_boards/gimbal_to_arm.hpp)
- [F/G 处理、打包、CRC 和 UART7 DMA 发送](../communication_between_boards/gimbal_to_arm.cpp)
- [任务循环读取遥控器并调用发送](../RtosTask/remote_control_task.cpp)
- [UART7 参数和引脚初始化](../Core/Src/usart.c)
