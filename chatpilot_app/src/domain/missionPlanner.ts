import {
  FlightControllerPayload,
  MissionCommand,
  MissionCommandCode,
  MissionPlan,
  PlannerEngine,
} from './types';

const DEFAULT_ALTITUDE_M = 1.2;
const DEFAULT_DISTANCE_M = 8;
const DEFAULT_SPEED_MPS = 0.6;

export const missionExamples = [
  '我在无人机上放了一个药盒子，你帮我送到前面再返回',
  '起飞到一米二，向前飞 10 米，拍照，然后返航降落',
  '把急救包送到左前方 6 米，放下后回来',
];

type Direction = {
  label: string;
  xM: number;
  yM: number;
};

function createMissionId() {
  const stamp = new Date().toISOString().replace(/[-:.TZ]/g, '').slice(0, 14);
  return `CP-${stamp}`;
}

function clamp(value: number, min: number, max: number) {
  return Math.min(Math.max(value, min), max);
}

function findDistance(text: string) {
  const match = text.match(/(\d+(?:\.\d+)?)\s*(米|m|M)/);
  if (!match) return DEFAULT_DISTANCE_M;
  return clamp(Number(match[1]), 1, 30);
}

function findAltitude(text: string) {
  const meterMatch = text.match(/(?:高度|高|起飞到|飞到)\s*(\d+(?:\.\d+)?)\s*(米|m|M)/);
  if (meterMatch) return clamp(Number(meterMatch[1]), 0.5, 5);

  if (text.includes('一米二') || text.includes('1米2') || text.includes('1.2米')) {
    return 1.2;
  }

  return DEFAULT_ALTITUDE_M;
}

function inferDirection(text: string, distanceM: number): Direction {
  const diagonalScale = Math.round((distanceM / Math.SQRT2) * 10) / 10;

  if (text.includes('左前')) return { label: '左前方', xM: diagonalScale, yM: -diagonalScale };
  if (text.includes('右前')) return { label: '右前方', xM: diagonalScale, yM: diagonalScale };
  if (text.includes('左')) return { label: '左侧', xM: 0, yM: -distanceM };
  if (text.includes('右')) return { label: '右侧', xM: 0, yM: distanceM };
  if (text.includes('后')) return { label: '后方', xM: -distanceM, yM: 0 };

  return { label: '前方', xM: distanceM, yM: 0 };
}

function inferPayload(text: string) {
  if (text.includes('药')) return '药盒';
  if (text.includes('急救')) return '急救包';
  if (text.includes('外卖')) return '外卖';
  if (text.includes('包裹')) return '包裹';
  if (text.includes('物资')) return '物资';
  return '载荷';
}

function command(
  commandCode: MissionCommandCode,
  title: string,
  detail: string,
  params: MissionCommand['params'],
  durationSec: number,
  mavlinkHint: string,
): Omit<MissionCommand, 'seq'> {
  return {
    command: commandCode,
    title,
    detail,
    params,
    durationSec,
    mavlinkHint,
  };
}

function resequence(commands: Array<Omit<MissionCommand, 'seq'>>): MissionCommand[] {
  return commands.map((item, index) => ({ ...item, seq: index + 1 }));
}

export function buildLocalMissionPlan(sourceText: string, engine: PlannerEngine = 'local-demo'): MissionPlan {
  const cleanText = sourceText.trim() || missionExamples[0];
  const distanceM = findDistance(cleanText);
  const altitudeM = findAltitude(cleanText);
  const direction = inferDirection(cleanText, distanceM);
  const payload = inferPayload(cleanText);
  const wantsReturn = /返|回|回来|返航/.test(cleanText);
  const wantsPhoto = /拍|照|巡检|查看|侦察/.test(cleanText);
  const wantsDelivery = /送|放|投|递|交付|药|包裹|物资|急救/.test(cleanText);

  const travelDuration = Math.ceil(distanceM / DEFAULT_SPEED_MPS);
  const commands: Array<Omit<MissionCommand, 'seq'>> = [
    command('ARM', '解锁电机', '进入任务待执行状态', { require_operator_confirm: true }, 1, 'MAV_CMD_COMPONENT_ARM_DISARM'),
    command(
      'TAKEOFF',
      '起飞定高',
      `爬升到 ${altitudeM.toFixed(1)} m，保持低速演示高度`,
      { altitude_m: altitudeM, climb_rate_mps: 0.4 },
      4,
      'MAV_CMD_NAV_TAKEOFF',
    ),
    command(
      'MOVE_BODY',
      `飞向${direction.label}`,
      `机体系位移 x=${direction.xM} m, y=${direction.yM} m`,
      { x_m: direction.xM, y_m: direction.yM, z_m: 0, speed_mps: DEFAULT_SPEED_MPS },
      travelDuration,
      'SET_POSITION_TARGET_LOCAL_NED',
    ),
  ];

  if (wantsPhoto) {
    commands.push(
      command('HOVER', '悬停稳定', '等待姿态收敛后执行拍照动作', { hold_sec: 2 }, 2, 'MAV_CMD_NAV_LOITER_TIME'),
      command('TAKE_PHOTO', '拍照记录', '记录目标点证明材料', { camera: 'demo_virtual_camera' }, 1, 'MAV_CMD_IMAGE_START_CAPTURE'),
    );
  }

  if (wantsDelivery) {
    commands.push(
      command('LAND', '目标点降落', '软降落到投递点', { landing_profile: 'gentle' }, 5, 'MAV_CMD_NAV_LAND'),
      command(
        'PAYLOAD_RELEASE',
        `释放${payload}`,
        '演示用舵机通道开合，不连接真实飞控',
        { payload, servo_channel: 7, pwm_us: 1800, demo_only: true },
        2,
        'MAV_CMD_DO_SET_SERVO',
      ),
    );
  } else {
    commands.push(command('HOVER', '到点悬停', '目标点保持位置', { hold_sec: 3 }, 3, 'MAV_CMD_NAV_LOITER_TIME'));
  }

  if (wantsReturn) {
    commands.push(
      command(
        'TAKEOFF',
        '再次起飞',
        `从目标点爬升到 ${altitudeM.toFixed(1)} m`,
        { altitude_m: altitudeM, climb_rate_mps: 0.4 },
        4,
        'MAV_CMD_NAV_TAKEOFF',
      ),
      command(
        'RETURN_HOME',
        '返航',
        '按起点相反位移返回 Home',
        { x_m: -direction.xM, y_m: -direction.yM, z_m: 0, speed_mps: DEFAULT_SPEED_MPS },
        travelDuration,
        'SET_POSITION_TARGET_LOCAL_NED / MAV_CMD_NAV_RETURN_TO_LAUNCH',
      ),
      command('LAND', '回到起点降落', '在 Home 点降落并结束任务', { landing_profile: 'home' }, 5, 'MAV_CMD_NAV_LAND'),
    );
  }

  commands.push(command('DISARM', '锁定电机', '任务结束，输出摘要日志', { reason: 'mission_complete' }, 1, 'MAV_CMD_COMPONENT_ARM_DISARM'));

  const sequenced = resequence(commands);
  const estimatedDurationSec = sequenced.reduce((sum, item) => sum + item.durationSec, 0);

  return {
    id: createMissionId(),
    sourceText: cleanText,
    intentSummary: wantsDelivery
      ? `携带${payload}，飞向${direction.label}${distanceM}米，完成投递${wantsReturn ? '并返回起点' : ''}`
      : `飞向${direction.label}${distanceM}米${wantsPhoto ? '并拍照' : ''}${wantsReturn ? '后返航' : ''}`,
    engine,
    confidence: cleanText.length > 8 ? 0.88 : 0.62,
    demoOnly: true,
    safety: {
      geofenceRadiusM: 30,
      maxAltitudeM: 5,
      requiresPilotConfirm: true,
    },
    commands: sequenced,
    estimatedDurationSec,
  };
}

export function toFlightControllerPayload(plan: MissionPlan): FlightControllerPayload {
  return {
    mission_id: plan.id,
    mode: 'DEMO_ONLY',
    source_text: plan.sourceText,
    intent_summary: plan.intentSummary,
    safety: plan.safety,
    commands: plan.commands.map((item) => ({
      seq: item.seq,
      command: item.command,
      params: item.params,
      mavlink_hint: item.mavlinkHint,
      duration_sec: item.durationSec,
    })),
  };
}

export function normalizeCloudMissionPlan(raw: unknown, sourceText: string): MissionPlan {
  if (!raw || typeof raw !== 'object') {
    return buildLocalMissionPlan(sourceText, 'cloud-llm');
  }

  const candidate = raw as Partial<MissionPlan>;
  const fallback = buildLocalMissionPlan(sourceText, 'cloud-llm');
  const rawCommands = Array.isArray(candidate.commands) ? candidate.commands : fallback.commands;

  const commands = rawCommands.map((item, index) => {
    const partial = item as Partial<MissionCommand>;
    return {
      seq: index + 1,
      command: partial.command ?? fallback.commands[index]?.command ?? 'HOVER',
      title: partial.title ?? fallback.commands[index]?.title ?? '任务步骤',
      detail: partial.detail ?? fallback.commands[index]?.detail ?? 'LLM 规划步骤',
      params: partial.params ?? fallback.commands[index]?.params ?? {},
      durationSec: Number(partial.durationSec ?? fallback.commands[index]?.durationSec ?? 2),
      mavlinkHint: partial.mavlinkHint ?? fallback.commands[index]?.mavlinkHint ?? 'CUSTOM_DEMO_COMMAND',
    } satisfies MissionCommand;
  });

  return {
    ...fallback,
    id: createMissionId(),
    intentSummary: candidate.intentSummary ?? fallback.intentSummary,
    confidence: clamp(Number(candidate.confidence ?? 0.82), 0, 0.99),
    engine: 'cloud-llm',
    commands,
    estimatedDurationSec: commands.reduce((sum, item) => sum + item.durationSec, 0),
  };
}
