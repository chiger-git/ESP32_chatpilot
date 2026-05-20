# Defense Checklist

答辩时可以按这个顺序展示，速度最快也最能说明“开源成果”。

## One-minute Opening

项目名称：ESP32 ChatPilot，低成本 ESP32 四旋翼 + 自然语言任务规划 App。

开源仓库：<https://github.com/chiger-git/ESP32_chatpilot>

核心闭环：

1. 复现 ESP32 迷你无人机飞控基础。
2. 接入 PMW3901 光流和 VL53L0X/VL53L1X 激光测距。
3. 用 ChatPilot App 将中文任务转换为任务计划和飞控 JSON。
4. 通过 Wi-Fi/MAVLink/串口调试链路连接飞控侧验证。

## What to Open on GitHub

| Page | Talking Point |
| --- | --- |
| `README.md` | 开源成果总览和运行方式 |
| `src/Flix_Main.cpp` | 飞控 250 Hz 独立任务和服务任务拆分 |
| `src/flow.cpp` | 光流 + 激光测距接入 |
| `src/mavlink.cpp` | Wi-Fi/MAVLink 通信 |
| `src/control.cpp` | 姿态/角速度控制、电机混控、安全限幅 |
| `chatpilot_app/App.tsx` | App 交互界面 |
| `chatpilot_app/src/domain/missionPlanner.ts` | 中文任务到指令序列的本地规划器 |
| `chatpilot_app/src/services/llmPlanner.ts` | OpenAI-compatible LLM 规划器 |
| `hardware/` | BOM、EDA、光流模块和可选语音模块资料 |

## Live Demo Script

1. 打开 App，输入“我在无人机上放了一个药盒子，你帮我送到前面再返回”。
2. 点击生成任务，展示任务时间线。
3. 展示飞控 JSON，强调每条命令都有 MAVLink hint。
4. 点击仿真执行，展示执行日志。
5. 切到固件串口，展示 `help`、`flow`、`wifi`、`rc` 等命令。

## Safety Statement

App 默认 `DEMO_ONLY`，不会直接驱动真实飞控；飞控侧所有电机测试、校准、调参都要求拆桨。这个设计是为了让自然语言规划可以安全演示，同时保留真实飞控接入扩展口。
