export type PlannerEngine = 'local-demo' | 'cloud-llm';

export type MissionCommandCode =
  | 'ARM'
  | 'TAKEOFF'
  | 'MOVE_BODY'
  | 'LAND'
  | 'PAYLOAD_RELEASE'
  | 'RETURN_HOME'
  | 'HOVER'
  | 'TAKE_PHOTO'
  | 'DISARM';

export type MissionCommand = {
  seq: number;
  command: MissionCommandCode;
  title: string;
  detail: string;
  params: Record<string, string | number | boolean>;
  durationSec: number;
  mavlinkHint: string;
};

export type MissionPlan = {
  id: string;
  sourceText: string;
  intentSummary: string;
  engine: PlannerEngine;
  confidence: number;
  demoOnly: true;
  safety: {
    geofenceRadiusM: number;
    maxAltitudeM: number;
    requiresPilotConfirm: boolean;
  };
  commands: MissionCommand[];
  estimatedDurationSec: number;
};

export type FlightControllerPayload = {
  mission_id: string;
  mode: 'DEMO_ONLY';
  source_text: string;
  intent_summary: string;
  safety: MissionPlan['safety'];
  commands: Array<{
    seq: number;
    command: MissionCommandCode;
    params: MissionCommand['params'];
    mavlink_hint: string;
    duration_sec: number;
  }>;
};

export type LlmConfig = {
  endpoint: string;
  apiKey: string;
  model: string;
};
