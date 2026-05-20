# Hardware Notes

本项目硬件以嘉立创开源项目「ESP32 迷你无人机」为基础，在其上接入光流、激光测距、SBUS 遥控、Wi-Fi/MAVLink 调试和可选语音模块资料。

## Included Files

| Path | Description |
| --- | --- |
| `hardware/jlc-esp32-mini-drone/BOM_ESP32迷你无人机.xlsx` | 基础飞控 BOM |
| `hardware/jlc-esp32-mini-drone/ESP32迷你无人机.eprj2` | 嘉立创 EDA 工程 |
| `hardware/jlc-esp32-mini-drone/ProPrj_ESP32迷你无人机_2026-02-15.epro2` | 专业版工程文件 |
| `hardware/jlc-esp32-mini-drone/New Project_2026-03-14_23-57-05.eprj2` | 追加模块/验证工程 |
| `hardware/atk-pmw3901-flow/ATK-PMW3901光流模块用户手册（兼容2m和4m激光传感器）--.pdf` | 光流模块手册 |
| `hardware/atk-pmw3901-flow/PMW3901 V1.1--.pdf` | 光流模块原理图 |
| `hardware/asr-pro-optional/ASR-PRO核心板原理图.png` | 可选离线语音模块原理图 |
| `hardware/asr-pro-optional/ASRPRO核心板规格书V1.1.pdf` | 可选离线语音模块规格书 |

未提交的资料：超过 100 MB 的 PDF、厂商 IDE/烧录工具安装包、可从厂商或原项目重新下载的大型压缩包。这样可以保证 GitHub 普通仓库可直接推送和浏览。

## Pin Map

| Module | Signal | ESP32 Pin | Code |
| --- | --- | --- | --- |
| Motor RL | PWM | GPIO12 | `MOTOR_0_PIN` |
| Motor RR | PWM | GPIO13 | `MOTOR_1_PIN` |
| Motor FR | PWM | GPIO15 | `MOTOR_2_PIN` |
| Motor FL | PWM | GPIO14 | `MOTOR_3_PIN` |
| SBUS Receiver | RX | GPIO16 | `Serial2` |
| SBUS Receiver | TX | GPIO17 | `Serial2` |
| PMW3901 | SPI SCK | GPIO18 | `FLOW_SPI_SCK` |
| PMW3901 | SPI MISO | GPIO19 | `FLOW_SPI_MISO` |
| PMW3901 | SPI MOSI | GPIO23 | `FLOW_SPI_MOSI` |
| PMW3901 | CS | GPIO25 | `FLOW_PMW_CS` |
| Flow Module | Power enable | GPIO27 | `FLOW_POWER_EN` |
| VL53L0X/VL53L1X | I2C SDA | GPIO21 | `FLOW_I2C_SDA` |
| VL53L0X/VL53L1X | I2C SCL | GPIO22 | `FLOW_I2C_SCL` |
| Battery ADC | ADC | GPIO36 | `BATTERY_ADC_PIN` |

## Firmware Dependencies

PlatformIO 会从 `platformio.ini` 拉取以下主要依赖：

- `okalachev/FlixPeriph`
- `okalachev/MAVLink`
- `mikeshub/FUTABA_SBUS`
- `bitcraze/Bitcraze PMW3901`
- `pololu/VL53L0X`
- `pololu/VL53L1X`

## Bring-up Checklist

1. 拆下所有螺旋桨。
2. 烧录固件并打开 115200 串口。
3. 执行 `ca` 完成 IMU 校准。
4. 执行 `cr` 完成 SBUS 通道校准。
5. 执行 `flowinit` 和 `flowtest 10` 验证 PMW3901/VL53 数据。
6. 执行 `mfr/mfl/mrr/mrl` 单电机检查方向和编号。
7. 执行 `rctest 0.2 5` 观察电机噪声下 RC 丢帧情况。
8. 确认安全锁定、失控保护和电池电压读取正常后再进入带桨测试。

## Source Links

- 嘉立创开源硬件平台项目：<https://oshwhub.com/malagis/esp32-mini-plane>
- Flix upstream reference：<https://github.com/okalachev/flix>
