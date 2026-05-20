# ChatPilot App

跨 Android / iOS 的 Expo 演示 App，用自然语言生成无人机任务计划。

## 快速运行

```powershell
cd chatpilot_app
npm run start
```

然后用手机安装 Expo Go，扫描终端里的二维码即可演示。

手机和电脑不在同一个局域网时，可以用：

```powershell
npm run start -- --tunnel
```

## 现场演示链路

1. 输入一句自然语言任务，例如“我在无人机上放了一个药盒子，你帮我送到前面再返回”。
2. 点击“生成任务”。
3. App 展示任务编排、飞控 JSON 和仿真执行日志。
4. 点击“仿真执行”，逐条播放指令，但不会连接真实飞控。

## LLM 模式

默认使用本地演示解析器，断网也能跑。打开“云端 LLM”开关后，可填写 OpenAI-compatible chat completions endpoint、model 和 API key。LLM 请求失败时会自动回退到本地演示解析器。

手机浏览器打开 Web 版时，建议先在电脑启动本地代理，避免浏览器跨域限制：

```powershell
npm run llm-proxy
```

手机页面里填写：

- Endpoint: `http://电脑热点IP:8787/v1/chat/completions`
- Model: `gpt-4o-mini`
- API Key: OpenAI Platform 生成的 `sk-...` 或 `sk-proj-...` key
