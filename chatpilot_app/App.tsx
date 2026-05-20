import { StatusBar } from 'expo-status-bar';
import * as Clipboard from 'expo-clipboard';
import {
  ActivityIndicator,
  KeyboardAvoidingView,
  Platform,
  Pressable,
  ScrollView,
  StyleSheet,
  Switch,
  Text,
  TextInput,
  View,
} from 'react-native';
import {
  Bot,
  CheckCircle2,
  ClipboardCopy,
  Cloud,
  Cpu,
  Plane,
  Play,
  RadioTower,
  Route,
  Send,
  ShieldCheck,
  TerminalSquare,
} from 'lucide-react-native';
import { useMemo, useState } from 'react';

import {
  buildLocalMissionPlan,
  missionExamples,
  toFlightControllerPayload,
} from './src/domain/missionPlanner';
import { LlmConfig, MissionPlan } from './src/domain/types';
import { planMissionWithLlm } from './src/services/llmPlanner';

const initialText = missionExamples[0];
const getDefaultEndpoint = () => {
  if (Platform.OS === 'web' && typeof window !== 'undefined' && window.location.hostname) {
    return `${window.location.protocol}//${window.location.hostname}:8787/v1/chat/completions`;
  }

  return 'https://api.openai.com/v1/chat/completions';
};

const defaultLlmConfig: LlmConfig = {
  endpoint: process.env.EXPO_PUBLIC_LLM_ENDPOINT ?? getDefaultEndpoint(),
  apiKey: process.env.EXPO_PUBLIC_LLM_API_KEY ?? '',
  model: process.env.EXPO_PUBLIC_LLM_MODEL ?? 'gpt-4o-mini',
};

const palette = {
  ink: '#1F2328',
  muted: '#69707D',
  line: '#D7DCE2',
  surface: '#FFFFFF',
  canvas: '#F4F7FA',
  green: '#0C8F68',
  blue: '#1C6DD0',
  amber: '#B86E00',
  red: '#C83F31',
  graphite: '#30343B',
};

function delay(ms: number) {
  return new Promise((resolve) => setTimeout(resolve, ms));
}

function Pill({
  label,
  tone = 'neutral',
}: {
  label: string;
  tone?: 'neutral' | 'green' | 'amber' | 'blue';
}) {
  return (
    <View style={[styles.pill, styles[`pill_${tone}`]]}>
      <Text style={[styles.pillText, styles[`pillText_${tone}`]]}>{label}</Text>
    </View>
  );
}

function IconButton({
  label,
  icon,
  onPress,
  primary,
  disabled,
}: {
  label: string;
  icon: React.ReactNode;
  onPress: () => void;
  primary?: boolean;
  disabled?: boolean;
}) {
  return (
    <Pressable
      accessibilityRole="button"
      disabled={disabled}
      onPress={onPress}
      style={({ pressed }) => [
        styles.button,
        primary ? styles.buttonPrimary : styles.buttonSecondary,
        pressed && !disabled ? styles.buttonPressed : null,
        disabled ? styles.buttonDisabled : null,
      ]}
    >
      {icon}
      <Text style={[styles.buttonText, primary ? styles.buttonTextPrimary : styles.buttonTextSecondary]}>{label}</Text>
    </Pressable>
  );
}

export default function App() {
  const [input, setInput] = useState(initialText);
  const [plan, setPlan] = useState<MissionPlan>(() => buildLocalMissionPlan(initialText));
  const [useLlm, setUseLlm] = useState(false);
  const [llmConfig, setLlmConfig] = useState(defaultLlmConfig);
  const [isPlanning, setIsPlanning] = useState(false);
  const [isSimulating, setIsSimulating] = useState(false);
  const [notice, setNotice] = useState('飞控链路：演示隔离');
  const [executionLog, setExecutionLog] = useState<string[]>([
    'SYSTEM boot: ChatPilot demo console ready',
    'LINK guard: no hardware command will be transmitted',
  ]);

  const payload = useMemo(() => toFlightControllerPayload(plan), [plan]);
  const payloadText = useMemo(() => JSON.stringify(payload, null, 2), [payload]);

  async function handlePlan() {
    setIsPlanning(true);
    setNotice(useLlm ? '正在请求 LLM 规划' : '正在本地解析任务');
    try {
      const nextPlan = useLlm
        ? await planMissionWithLlm(input, llmConfig)
        : buildLocalMissionPlan(input, 'local-demo');
      setPlan(nextPlan);
      setExecutionLog((items) => [
        `PLAN ${nextPlan.id}: ${nextPlan.intentSummary}`,
        `ENGINE ${nextPlan.engine}: ${nextPlan.commands.length} commands`,
        ...items.slice(0, 6),
      ]);
      setNotice(nextPlan.engine === 'cloud-llm' ? 'LLM 规划完成' : '本地演示规划完成');
    } catch (error) {
      const fallback = buildLocalMissionPlan(input, 'local-demo');
      const message = (error as Error).message;
      setPlan(fallback);
      setNotice(`LLM 不可用：${message.slice(0, 46)}`);
      setExecutionLog((items) => [
        `FALLBACK: ${message}`,
        `PLAN ${fallback.id}: ${fallback.intentSummary}`,
        ...items.slice(0, 6),
      ]);
    } finally {
      setIsPlanning(false);
    }
  }

  async function handleCopy() {
    await Clipboard.setStringAsync(payloadText);
    setNotice('任务 JSON 已复制');
  }

  async function handleSimulate() {
    if (isSimulating) return;
    setIsSimulating(true);
    setExecutionLog([`MISSION ${plan.id}: armed in demo sandbox`]);

    for (const item of plan.commands) {
      await delay(360);
      setExecutionLog((items) => [
        `[${String(item.seq).padStart(2, '0')}] ${item.command} ${JSON.stringify(item.params)}`,
        ...items,
      ]);
    }

    await delay(300);
    setExecutionLog((items) => ['MISSION complete: no packets sent to real FC', ...items]);
    setNotice('演示执行完成');
    setIsSimulating(false);
  }

  return (
    <KeyboardAvoidingView
      behavior={Platform.OS === 'ios' ? 'padding' : undefined}
      style={styles.root}
    >
      <StatusBar style="dark" />
      <ScrollView
        contentContainerStyle={styles.scrollContent}
        keyboardShouldPersistTaps="handled"
        showsVerticalScrollIndicator={false}
      >
        <View style={styles.header}>
          <View style={styles.brandMark}>
            <Plane color={palette.surface} size={25} strokeWidth={2.4} />
          </View>
          <View style={styles.headerText}>
            <Text style={styles.appName}>ChatPilot</Text>
            <Text style={styles.appSubtitle}>自然语言飞控任务编排</Text>
          </View>
          <Pill label="DEMO" tone="green" />
        </View>

        <View style={styles.statusRail}>
          <View style={styles.statusItem}>
            <RadioTower color={palette.green} size={19} />
            <Text style={styles.statusText}>{notice}</Text>
          </View>
          <View style={styles.statusItem}>
            <ShieldCheck color={palette.blue} size={19} />
            <Text style={styles.statusText}>真实飞控未接入</Text>
          </View>
        </View>

        <View style={styles.panel}>
          <View style={styles.panelTitleRow}>
            <Bot color={palette.green} size={22} />
            <Text style={styles.panelTitle}>自然语言任务</Text>
          </View>

          <TextInput
            multiline
            value={input}
            onChangeText={setInput}
            placeholder="我在无人机上放了一个药盒子，你帮我送到前面再返回"
            placeholderTextColor="#8C949F"
            style={styles.promptInput}
            textAlignVertical="top"
          />

          <View style={styles.exampleRow}>
            {missionExamples.map((example) => (
              <Pressable
                key={example}
                onPress={() => setInput(example)}
                style={({ pressed }) => [styles.exampleChip, pressed ? styles.exampleChipPressed : null]}
              >
                <Text numberOfLines={2} style={styles.exampleText}>{example}</Text>
              </Pressable>
            ))}
          </View>

          <View style={styles.engineRow}>
            <View style={styles.engineLeft}>
              {useLlm ? <Cloud color={palette.blue} size={20} /> : <Cpu color={palette.green} size={20} />}
              <Text style={styles.engineLabel}>{useLlm ? '云端 LLM' : '本地演示解析'}</Text>
            </View>
            <Switch
              value={useLlm}
              onValueChange={setUseLlm}
              trackColor={{ false: '#C9D2DC', true: '#9DC8FF' }}
              thumbColor={useLlm ? palette.blue : '#FFFFFF'}
            />
          </View>

          {useLlm ? (
            <View style={styles.llmBox}>
              <TextInput
                value={llmConfig.endpoint}
                onChangeText={(endpoint) => setLlmConfig((value) => ({ ...value, endpoint }))}
                autoCapitalize="none"
                autoCorrect={false}
                placeholder="Endpoint"
                style={styles.compactInput}
              />
              <TextInput
                value={llmConfig.model}
                onChangeText={(model) => setLlmConfig((value) => ({ ...value, model }))}
                autoCapitalize="none"
                autoCorrect={false}
                placeholder="Model"
                style={styles.compactInput}
              />
              <TextInput
                value={llmConfig.apiKey}
                onChangeText={(apiKey) => setLlmConfig((value) => ({ ...value, apiKey }))}
                autoCapitalize="none"
                autoCorrect={false}
                placeholder="API Key"
                secureTextEntry
                style={styles.compactInput}
              />
            </View>
          ) : null}

          <IconButton
            disabled={isPlanning}
            icon={isPlanning ? <ActivityIndicator color={palette.surface} /> : <Send color={palette.surface} size={19} />}
            label={isPlanning ? '规划中' : '生成任务'}
            onPress={handlePlan}
            primary
          />
        </View>

        <View style={styles.metricsGrid}>
          <View style={styles.metricCell}>
            <Text style={styles.metricValue}>{plan.commands.length}</Text>
            <Text style={styles.metricLabel}>指令</Text>
          </View>
          <View style={styles.metricCell}>
            <Text style={styles.metricValue}>{plan.estimatedDurationSec}s</Text>
            <Text style={styles.metricLabel}>估计时长</Text>
          </View>
          <View style={styles.metricCell}>
            <Text style={styles.metricValue}>{Math.round(plan.confidence * 100)}%</Text>
            <Text style={styles.metricLabel}>置信度</Text>
          </View>
        </View>

        <View style={styles.panel}>
          <View style={styles.panelTitleRow}>
            <Route color={palette.amber} size={22} />
            <Text style={styles.panelTitle}>任务编排</Text>
            <Pill label={plan.engine === 'cloud-llm' ? 'LLM' : 'LOCAL'} tone={plan.engine === 'cloud-llm' ? 'blue' : 'green'} />
          </View>
          <Text style={styles.intentText}>{plan.intentSummary}</Text>

          <View style={styles.timeline}>
            {plan.commands.map((item, index) => (
              <View key={`${item.seq}-${item.command}`} style={styles.timelineRow}>
                <View style={styles.timelineRail}>
                  <View style={styles.timelineDot}>
                    <Text style={styles.timelineIndex}>{item.seq}</Text>
                  </View>
                  {index < plan.commands.length - 1 ? <View style={styles.timelineLine} /> : null}
                </View>
                <View style={styles.commandCard}>
                  <View style={styles.commandHeader}>
                    <Text style={styles.commandTitle}>{item.title}</Text>
                    <Text style={styles.commandCode}>{item.command}</Text>
                  </View>
                  <Text style={styles.commandDetail}>{item.detail}</Text>
                  <Text style={styles.mavlinkText}>{item.mavlinkHint}</Text>
                </View>
              </View>
            ))}
          </View>
        </View>

        <View style={styles.actionRow}>
          <IconButton
            disabled={isSimulating}
            icon={isSimulating ? <ActivityIndicator color={palette.surface} /> : <Play color={palette.surface} size={19} />}
            label={isSimulating ? '执行中' : '仿真执行'}
            onPress={handleSimulate}
            primary
          />
          <IconButton
            icon={<ClipboardCopy color={palette.green} size={19} />}
            label="复制 JSON"
            onPress={handleCopy}
          />
        </View>

        <View style={styles.panel}>
          <View style={styles.panelTitleRow}>
            <TerminalSquare color={palette.graphite} size={22} />
            <Text style={styles.panelTitle}>飞控 JSON</Text>
            <Pill label="只读" tone="amber" />
          </View>
          <ScrollView horizontal showsHorizontalScrollIndicator={false} style={styles.codeScroll}>
            <Text selectable style={styles.codeText}>{payloadText}</Text>
          </ScrollView>
        </View>

        <View style={styles.panel}>
          <View style={styles.panelTitleRow}>
            <CheckCircle2 color={palette.green} size={22} />
            <Text style={styles.panelTitle}>执行日志</Text>
          </View>
          {executionLog.slice(0, 9).map((line) => (
            <Text key={line} style={styles.logLine}>{line}</Text>
          ))}
        </View>
      </ScrollView>
    </KeyboardAvoidingView>
  );
}

const styles = StyleSheet.create({
  root: {
    flex: 1,
    backgroundColor: palette.canvas,
  },
  scrollContent: {
    paddingHorizontal: 18,
    paddingBottom: 34,
    paddingTop: Platform.select({ ios: 62, android: 38, default: 34 }),
  },
  header: {
    alignItems: 'center',
    flexDirection: 'row',
    gap: 12,
    marginBottom: 18,
  },
  brandMark: {
    alignItems: 'center',
    backgroundColor: palette.green,
    borderRadius: 8,
    height: 46,
    justifyContent: 'center',
    width: 46,
  },
  headerText: {
    flex: 1,
    minWidth: 0,
  },
  appName: {
    color: palette.ink,
    fontSize: 27,
    fontWeight: '800',
    letterSpacing: 0,
  },
  appSubtitle: {
    color: palette.muted,
    fontSize: 13,
    marginTop: 2,
  },
  pill: {
    alignItems: 'center',
    borderRadius: 8,
    borderWidth: 1,
    justifyContent: 'center',
    minHeight: 28,
    paddingHorizontal: 10,
  },
  pill_neutral: {
    backgroundColor: '#EEF1F4',
    borderColor: '#D8DEE6',
  },
  pill_green: {
    backgroundColor: '#E7F6EF',
    borderColor: '#A7DCC5',
  },
  pill_amber: {
    backgroundColor: '#FFF3D7',
    borderColor: '#E8C46D',
  },
  pill_blue: {
    backgroundColor: '#E8F1FF',
    borderColor: '#B7D4FF',
  },
  pillText: {
    fontSize: 11,
    fontWeight: '800',
  },
  pillText_neutral: {
    color: palette.graphite,
  },
  pillText_green: {
    color: palette.green,
  },
  pillText_amber: {
    color: palette.amber,
  },
  pillText_blue: {
    color: palette.blue,
  },
  statusRail: {
    backgroundColor: palette.surface,
    borderColor: palette.line,
    borderRadius: 8,
    borderWidth: 1,
    gap: 10,
    marginBottom: 14,
    padding: 12,
  },
  statusItem: {
    alignItems: 'center',
    flexDirection: 'row',
    gap: 9,
  },
  statusText: {
    color: palette.graphite,
    flex: 1,
    fontSize: 13,
  },
  panel: {
    backgroundColor: palette.surface,
    borderColor: palette.line,
    borderRadius: 8,
    borderWidth: 1,
    marginBottom: 14,
    padding: 14,
  },
  panelTitleRow: {
    alignItems: 'center',
    flexDirection: 'row',
    gap: 9,
    marginBottom: 12,
  },
  panelTitle: {
    color: palette.ink,
    flex: 1,
    fontSize: 17,
    fontWeight: '800',
  },
  promptInput: {
    backgroundColor: '#F8FAFC',
    borderColor: '#CAD2DD',
    borderRadius: 8,
    borderWidth: 1,
    color: palette.ink,
    fontSize: 16,
    lineHeight: 23,
    minHeight: 104,
    padding: 12,
  },
  exampleRow: {
    gap: 8,
    marginTop: 10,
  },
  exampleChip: {
    backgroundColor: '#F0F4F7',
    borderColor: '#D9E0E8',
    borderRadius: 8,
    borderWidth: 1,
    minHeight: 42,
    paddingHorizontal: 11,
    paddingVertical: 8,
  },
  exampleChipPressed: {
    backgroundColor: '#E5EEF6',
  },
  exampleText: {
    color: palette.graphite,
    fontSize: 13,
    lineHeight: 18,
  },
  engineRow: {
    alignItems: 'center',
    flexDirection: 'row',
    justifyContent: 'space-between',
    marginTop: 12,
  },
  engineLeft: {
    alignItems: 'center',
    flexDirection: 'row',
    gap: 8,
  },
  engineLabel: {
    color: palette.ink,
    fontSize: 14,
    fontWeight: '700',
  },
  llmBox: {
    gap: 8,
    marginTop: 10,
  },
  compactInput: {
    backgroundColor: '#F8FAFC',
    borderColor: '#CAD2DD',
    borderRadius: 8,
    borderWidth: 1,
    color: palette.ink,
    fontSize: 13,
    minHeight: 42,
    paddingHorizontal: 11,
  },
  button: {
    alignItems: 'center',
    borderRadius: 8,
    flexDirection: 'row',
    gap: 8,
    justifyContent: 'center',
    minHeight: 48,
    paddingHorizontal: 14,
  },
  buttonPrimary: {
    backgroundColor: palette.green,
    marginTop: 14,
  },
  buttonSecondary: {
    backgroundColor: '#EAF4EF',
    borderColor: '#B8DDCA',
    borderWidth: 1,
    flex: 1,
  },
  buttonPressed: {
    opacity: 0.82,
  },
  buttonDisabled: {
    opacity: 0.62,
  },
  buttonText: {
    fontSize: 15,
    fontWeight: '800',
  },
  buttonTextPrimary: {
    color: palette.surface,
  },
  buttonTextSecondary: {
    color: palette.green,
  },
  metricsGrid: {
    flexDirection: 'row',
    gap: 10,
    marginBottom: 14,
  },
  metricCell: {
    backgroundColor: palette.surface,
    borderColor: palette.line,
    borderRadius: 8,
    borderWidth: 1,
    flex: 1,
    minHeight: 76,
    padding: 10,
  },
  metricValue: {
    color: palette.ink,
    fontSize: 22,
    fontWeight: '900',
  },
  metricLabel: {
    color: palette.muted,
    fontSize: 12,
    marginTop: 5,
  },
  intentText: {
    color: palette.graphite,
    fontSize: 14,
    lineHeight: 21,
    marginBottom: 8,
  },
  timeline: {
    gap: 0,
  },
  timelineRow: {
    flexDirection: 'row',
    minHeight: 78,
  },
  timelineRail: {
    alignItems: 'center',
    width: 34,
  },
  timelineDot: {
    alignItems: 'center',
    backgroundColor: '#E8F1FF',
    borderColor: '#AFCFFF',
    borderRadius: 8,
    borderWidth: 1,
    height: 28,
    justifyContent: 'center',
    width: 28,
  },
  timelineIndex: {
    color: palette.blue,
    fontSize: 12,
    fontWeight: '900',
  },
  timelineLine: {
    backgroundColor: '#CBD8E6',
    flex: 1,
    width: 2,
  },
  commandCard: {
    backgroundColor: '#F9FBFD',
    borderColor: '#DEE5ED',
    borderRadius: 8,
    borderWidth: 1,
    flex: 1,
    marginBottom: 10,
    padding: 10,
  },
  commandHeader: {
    alignItems: 'flex-start',
    flexDirection: 'row',
    gap: 8,
  },
  commandTitle: {
    color: palette.ink,
    flex: 1,
    fontSize: 15,
    fontWeight: '800',
    lineHeight: 20,
  },
  commandCode: {
    color: palette.blue,
    flexShrink: 1,
    fontSize: 11,
    fontWeight: '900',
  },
  commandDetail: {
    color: palette.graphite,
    fontSize: 13,
    lineHeight: 19,
    marginTop: 5,
  },
  mavlinkText: {
    color: palette.muted,
    fontSize: 11,
    marginTop: 7,
  },
  actionRow: {
    flexDirection: 'row',
    gap: 10,
    marginBottom: 14,
  },
  codeScroll: {
    backgroundColor: '#20242A',
    borderRadius: 8,
    maxHeight: 260,
    padding: 12,
  },
  codeText: {
    color: '#ECF3F1',
    fontFamily: Platform.select({ ios: 'Menlo', android: 'monospace', default: 'monospace' }),
    fontSize: 12,
    lineHeight: 18,
    minWidth: 640,
  },
  logLine: {
    backgroundColor: '#F6F8FA',
    borderColor: '#E1E6EC',
    borderRadius: 8,
    borderWidth: 1,
    color: palette.graphite,
    fontFamily: Platform.select({ ios: 'Menlo', android: 'monospace', default: 'monospace' }),
    fontSize: 12,
    lineHeight: 17,
    marginBottom: 7,
    paddingHorizontal: 9,
    paddingVertical: 7,
  },
});
