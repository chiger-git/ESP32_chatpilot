## ESP32 ChatPilot Roadmap

**TL;DR**

项目目标是把低成本 ESP32 四旋翼、光流/测距感知、MAVLink 调试链路和 LLM 自然语言任务规划组合成一个可答辩、可复现、可继续扩展的开源原型。

## Stage Status

| Stage | Status | Evidence |
| --- | --- | --- |
| 复现基础飞控 | Done | `src/Flix_Main.cpp`, `src/control.cpp`, `src/motors.cpp`, `src/rc.cpp` |
| 接入光流 + 激光测距 | Done for bench validation | `src/flow.cpp`, `flowinit`, `flow`, `flowtest` |
| 接入 LLM 任务规划 | Done for demo workflow | `chatpilot_app/`, local planner, OpenAI-compatible planner |
| Agent 接入真实飞控 | Interface prepared | App exports FC JSON, firmware has Wi-Fi/MAVLink/CLI hooks |
| 开源交付 | Done | `README.md`, `hardware/`, `docs/`, `LICENSE`, `NOTICE.md` |

## Current Architecture

1. 飞控固件运行 250 Hz 独立控制任务，服务任务负责 CLI、MAVLink、参数同步。
2. SBUS 遥控输入经过校准和诊断，失控保护会记录 RC 快照。
3. 姿态控制采用稳定模式为主，包含角速度 PID、怠速补偿、电池电压补偿和电机微调。
4. 光流模块使用 PMW3901，测距模块兼容 VL53L0X/VL53L1X，用于近地感知验证。
5. ChatPilot App 将中文任务转换为指令序列、MAVLink hint 和飞控 JSON，默认 `DEMO_ONLY`。

## Next Field Work

1. 将 App 生成的任务 JSON 通过 UDP/MAVLink 桥接到飞控命令处理器。
2. 用光流位移和 ToF 高度估计替换当前 bench 读取，形成低速室内定位观测量。
3. 增加任务级状态机：起飞、定点移动、悬停、投放、返航、降落。
4. 以拆桨台架、吊线保护、低油门限幅三步完成实机闭环验证。
