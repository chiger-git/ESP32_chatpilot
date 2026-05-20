import { normalizeCloudMissionPlan } from '../domain/missionPlanner';
import { LlmConfig, MissionPlan } from '../domain/types';

function extractJson(text: string) {
  const trimmed = text.trim();
  if (trimmed.startsWith('{') && trimmed.endsWith('}')) return trimmed;

  const start = trimmed.indexOf('{');
  const end = trimmed.lastIndexOf('}');
  if (start >= 0 && end > start) return trimmed.slice(start, end + 1);

  throw new Error('LLM response did not contain JSON');
}

export async function planMissionWithLlm(sourceText: string, config: LlmConfig): Promise<MissionPlan> {
  if (!config.apiKey.trim()) {
    throw new Error('Missing API key');
  }

  const response = await fetch(config.endpoint.trim(), {
    method: 'POST',
    headers: {
      Authorization: `Bearer ${config.apiKey.trim()}`,
      'Content-Type': 'application/json',
    },
    body: JSON.stringify({
      model: config.model.trim(),
      temperature: 0.2,
      response_format: { type: 'json_object' },
      messages: [
        {
          role: 'system',
          content:
            '你是无人机任务规划器。只输出 JSON。把中文自然语言转换成演示用飞控任务，不要真的连接硬件。commands 只能使用 ARM, TAKEOFF, MOVE_BODY, LAND, PAYLOAD_RELEASE, RETURN_HOME, HOVER, TAKE_PHOTO, DISARM。每个 command 需要 title, detail, params, durationSec, mavlinkHint。',
        },
        {
          role: 'user',
          content: JSON.stringify({
            sourceText,
            outputShape: {
              intentSummary: '中文摘要',
              confidence: 0.0,
              commands: [
                {
                  command: 'TAKEOFF',
                  title: '起飞定高',
                  detail: '爬升到指定高度',
                  params: { altitude_m: 1.2 },
                  durationSec: 4,
                  mavlinkHint: 'MAV_CMD_NAV_TAKEOFF',
                },
              ],
            },
          }),
        },
      ],
    }),
  });

  if (!response.ok) {
    let detail = '';
    try {
      const errorBody = await response.json();
      detail = errorBody?.error?.message || errorBody?.error || '';
    } catch {
      detail = await response.text();
    }

    throw new Error(`LLM request failed ${response.status}${detail ? `: ${detail}` : ''}`);
  }

  const json = await response.json();
  const content = json?.choices?.[0]?.message?.content;
  if (typeof content !== 'string') {
    throw new Error('LLM response missing message content');
  }

  return normalizeCloudMissionPlan(JSON.parse(extractJson(content)), sourceText);
}
