# ESP32 ChatPilot

ESP32 ChatPilot 是一个面向低成本四旋翼无人机的开源演示项目：底层使用 ESP32/Arduino/PlatformIO 飞控固件，上层使用 Expo App 将中文自然语言任务转换为无人机任务计划，并输出飞控侧可消费的 JSON/MAVLink 指令提示。

本仓库用于比赛答辩开源展示，重点覆盖三部分成果：

- `src/`：ESP32 飞控固件，基于 Flix 思路改造，包含 250 Hz 飞控任务、SBUS 遥控、姿态/角速度控制、电机混控、Wi-Fi UDP/MAVLink、光流/激光测距接入测试、RC 诊断、自动电机微调和安全保护。
- `chatpilot_app/`：ChatPilot Expo App，支持本地演示解析器和 OpenAI-compatible LLM 两种规划模式，可把中文任务生成任务编排、飞控 JSON 和仿真执行日志。
- `hardware/`：嘉立创开源项目 BOM/EDA 文件、ATK-PMW3901 光流模块资料，以及可选语音模块资料，便于复现硬件连接。

## Demo Flow

```mermaid
flowchart LR
  A["中文自然语言任务"] --> B["ChatPilot App"]
  B --> C["本地解析器或云端 LLM"]
  C --> D["Mission Plan"]
  D --> E["飞控 JSON / MAVLink Hint"]
  E --> F["ESP32 飞控调试链路"]
```

当前 App 默认运行在 `DEMO_ONLY` 安全模式：它展示任务规划、指令序列和执行日志，不会直接向真实飞控下发危险动作。飞控固件侧已保留 Wi-Fi/MAVLink 通信、CLI 和安全锁定逻辑，适合继续接入实际任务执行器。

## Repository Layout

| Path | Content |
| --- | --- |
| `src/` | ESP32 飞控主程序和各功能模块 |
| `platformio.ini` | PlatformIO 构建配置和依赖 |
| `chatpilot_app/` | React Native / Expo 自然语言任务规划 App |
| `hardware/` | BOM、EDA、模块手册、原理图 |
| `docs/` | 硬件说明和答辩展示清单 |
| `plan.md` | 项目路线图和阶段状态 |

## Firmware Quick Start

1. 安装 VS Code + PlatformIO，或使用 PlatformIO CLI。
2. 连接 ESP32 开发板。
3. 在仓库根目录执行：

```powershell
pio run
pio run -t upload
pio device monitor -b 115200
```

常用串口命令：

| Command | Purpose |
| --- | --- |
| `help` | 打印命令列表 |
| `cr` | 校准 SBUS 遥控通道 |
| `ca` | 校准 IMU 加速度计 |
| `flowinit` / `flow` | 初始化并读取 PMW3901 + VL53L0X/VL53L1X |
| `flowtest 10` | 光流/测距 10 秒测试 |
| `rctest 0.2 5` | 电机噪声下 RC 可靠性测试，必须拆桨 |
| `atrim` | 地面怠速电机微调，必须拆桨 |
| `wifi` | 打印 Wi-Fi / MAVLink 统计 |

## App Quick Start

```powershell
cd chatpilot_app
npm install
npm run start
```

手机安装 Expo Go 后扫描终端二维码即可演示。Web 或手机浏览器需要云端 LLM 时，可启动本地代理：

```powershell
npm run llm-proxy
```

可复制 `chatpilot_app/.env.example` 为本地 `.env`，填写 OpenAI-compatible endpoint、model 和 API key。`.env`、日志和构建产物已加入忽略规则，不会进入开源仓库。

## Hardware

硬件复现说明见 [docs/HARDWARE.md](docs/HARDWARE.md)。仓库内已放入必要 BOM、EDA 工程和模块说明资料；大体积厂商工具包和超过 GitHub 单文件限制的 PDF 没有提交，README 中保留了来源说明。

## Safety

飞控代码会直接控制电机。任何上电、校准、调参、测试前都必须拆下螺旋桨，并确认电池、电机方向、通道映射、解锁逻辑和失控保护正常。App 侧默认仅做演示规划，不应跳过飞控安全校验。

## Attribution and License

飞控基础代码保留了上游 Flix / ESP32 迷你无人机相关版权声明；本项目新增代码、文档和 ChatPilot App 代码按 MIT License 开源，第三方库、嘉立创开源硬件资料和厂商模块资料遵循其原始许可。详见 [NOTICE.md](NOTICE.md) 和 [LICENSE](LICENSE)。
