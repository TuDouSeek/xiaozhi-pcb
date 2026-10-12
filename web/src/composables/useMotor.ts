import { reactive, ref, onUnmounted } from 'vue';

// 电机测试台前端封装：走 HTTP API（DEV 经 Vite 同源代理，生产直连设备 IP）。
// 与雷达点云链路独立，1Hz 轮询遥测；控制为一次性 GET，下发即返。

export type MotorDir = 'forward' | 'backward' | 'stop' | 'brake';
export type MotorSel = 'A' | 'B' | 'both';

export interface MotorInfo {
  dir: number; // 0=stop 1=forward 2=backward 3=brake
  speed: number;
  rpm: number;
  cnt: number;
}

const DIR_NAME = ['停止', '正转', '反转', '刹车'];

function apiBase(): string {
  if (import.meta.env.DEV) return ''; // 同源，Vite 代理到机器人
  const ip = localStorage.getItem('robot_ip');
  return ip ? `http://${ip}` : '';
}

export function useMotor() {
  const motorA = reactive<MotorInfo>({ dir: 0, speed: 0, rpm: 0, cnt: 0 });
  const motorB = reactive<MotorInfo>({ dir: 0, speed: 0, rpm: 0, cnt: 0 });
  const sliderA = ref(50);
  const sliderB = ref(50);
  const motorOnline = ref(false);
  let timer: number | null = null;

  async function cmd(m: MotorSel, dir: MotorDir, speed: number) {
    try {
      await fetch(`${apiBase()}/api/motor/control?motor=${m}&dir=${dir}&speed=${speed}`);
      await poll();
    } catch { /* 离线时静默 */ }
  }

  // 滑块拖动：仅在正/反转方向下实时下发，避免拖滑块即启动
  async function slider(m: 'A' | 'B', dir: MotorDir) {
    if (dir !== 'forward' && dir !== 'backward') return;
    await cmd(m, dir, m === 'A' ? sliderA.value : sliderB.value);
  }

  async function poll() {
    try {
      const r = await fetch(`${apiBase()}/api/motor/status`);
      if (!r.ok) {
        motorOnline.value = false;
        return;
      }
      const j = await r.json();
      Object.assign(motorA, j.a);
      Object.assign(motorB, j.b);
      motorOnline.value = true;
    } catch {
      motorOnline.value = false;
    }
  }

  void poll();
  timer = window.setInterval(poll, 1000);
  onUnmounted(() => {
    if (timer !== null) clearInterval(timer);
  });

  return { motorA, motorB, sliderA, sliderB, motorOnline, cmd, slider, poll, DIR_NAME };
}
