<template>
  <div class="console">
    <header>
      <h1>小智履带机器人</h1>
      <div class="link" :class="{ on: connected }" :title="lastUrl">
        {{ connected ? `已连接 · ${rttMs >= 0 ? rttMs.toFixed(0) + 'ms' : '…'}` : '未连接' }}
      </div>
    </header>

    <section class="ipbar">
      <label>设备 IP</label>
      <input v-model="ipInput" placeholder="192.168.137.148" @keyup.enter="applyIp" />
      <button @click="applyIp">重新连接</button>
      <button class="ghost" @click="useAutoIp" title="生产环境跟随页面 Host">自动</button>
      <span class="url">{{ lastUrl || wsUrl }}</span>
    </section>
    <p v-if="lastError && !connected" class="err">{{ lastError }}（2.5s 后重试）</p>

    <main>
      <section class="radar">
        <div class="rangebar">
          <span>范围</span>
          <button
            v-for="r in ranges"
            :key="r"
            :class="{ on: rangeM === r }"
            @click="rangeM = r"
          >{{ r }}m</button>
        </div>
        <RadarCanvas :pts="lidar.pts" :max-range-mm="rangeM * 1000" />
      </section>

      <section class="side">
        <div class="cards">
          <div class="card"><small>前方安全距离</small><b>{{ fmt(lidar.front) }}<span>mm</span></b></div>
          <div class="card"><small>左 / 右均值</small><b>{{ fmt(lidar.left) }} / {{ fmt(lidar.right) }}</b></div>
          <div class="card"><small>最近障碍</small><b>{{ fmt(lidar.gmin) }}mm @{{ lidar.gangle }}°</b></div>
          <div class="card"><small>逃逸航向 / RPM</small><b>{{ lidar.escape }}° / {{ lidar.rpm }}</b></div>
          <div class="card"><small>编码器 L / R（预留）</small><b>{{ encL }} / {{ encR }}</b></div>
          <div class="card"><small>遥控输出 v / w</small><b>{{ outV }} / {{ outW }}</b></div>
        </div>

        <div class="teach">
          <button @click="teach('record')">开始录制路线</button>
          <button @click="teach('waypoint')">保存路点</button>
          <button @click="teach('navigate')">自主导航</button>
        </div>
        <p class="hint">桌面：WASD / 方向键遥控 · 触屏：左下摇杆 · 示教功能为预留接口</p>
      </section>
    </main>

    <footer>
      <VirtualJoystick @drive="onJoy" @end="onJoyEnd" />
      <button class="stop" @touchstart="stop" @click="stop">急停</button>
    </footer>

    <section class="motor">
      <h2>电机测试台 <small>{{ motorOnline ? '在线 @1Hz' : '离线' }}</small></h2>
      <div class="row">
        <button @click="mcmd('both', 'forward')">前进</button>
        <button @click="mcmd('both', 'backward')">后退</button>
        <button class="warn" @click="mcmd('both', 'stop')">停止</button>
        <button class="warn" @click="mcmd('both', 'brake')">刹车</button>
      </div>
      <div class="mgrid">
        <div class="mcard" v-for="m in (['A', 'B'] as const)" :key="m">
          <h3>电机 {{ m }}{{ m === 'A' ? '（左轮）' : '（右轮）' }}</h3>
          <div class="row">
            <button @click="mcmd(m, 'forward')">正转</button>
            <button @click="mcmd(m, 'backward')">反转</button>
            <button class="warn" @click="mcmd(m, 'brake')">刹车</button>
            <button class="warn" @click="mcmd(m, 'stop')">停止</button>
          </div>
          <label>速度 <b>{{ m === 'A' ? sliderA : sliderB }}%</b></label>
          <input
            type="range" min="0" max="100" :value="m === 'A' ? sliderA : sliderB"
            @input="onSlider(m, $event)" @change="onSliderGo(m)"
          />
          <div class="tele">{{ tele(m) }}</div>
        </div>
      </div>
    </section>
  </div>
</template>

<script setup lang="ts">
import { ref, onMounted, onBeforeUnmount } from 'vue';
import RadarCanvas from './components/RadarCanvas.vue';
import VirtualJoystick from './components/VirtualJoystick.vue';
import { useRobot } from './composables/useRobot';
import { useMotor, type MotorDir, type MotorSel } from './composables/useMotor';

const { lidar, connected, rttMs, lastUrl, lastError,
  targetIp, wsUrl, setDrive, stop, setIp, useAutoIp, reconnect } = useRobot();
const { motorA, motorB, sliderA, sliderB, motorOnline, cmd: mcmdRaw } = useMotor();
// 每个电机记忆最后方向：滑块拖动时沿用，避免拖滑块即启动
const lastDir = ref<Record<'A' | 'B', MotorDir>>({ A: 'forward', B: 'forward' });
function mcmd(m: MotorSel, d: MotorDir) {
  if (m === 'A' || m === 'B') lastDir.value[m] = d;
  const s = m === 'A' ? sliderA.value : m === 'B' ? sliderB.value : 50;
  void mcmdRaw(m, d, s);
}
function onSlider(m: 'A' | 'B', e: Event) {
  const v = Number((e.target as HTMLInputElement).value);
  if (m === 'A') sliderA.value = v;
  else sliderB.value = v;
}
function onSliderGo(m: 'A' | 'B') {
  const d = lastDir.value[m];
  if (d !== 'forward' && d !== 'backward') return;
  void mcmdRaw(m, d, m === 'A' ? sliderA.value : sliderB.value);
}
function tele(m: 'A' | 'B') {
  const t = m === 'A' ? motorA : motorB;
  return `rpm ${t.rpm} · cnt ${t.cnt} · ${['停止', '正转', '反转', '刹车'][t.dir] ?? '-'} ${t.speed}%`;
}
const encL = ref('-');
const encR = ref('-');
const outV = ref(0);
const outW = ref(0);
const ipInput = ref(targetIp.value);
// 雷达量程档位（半径）：Canvas 按 maxRangeMm 等比缩放
const ranges = [1, 2, 3, 6];
const rangeM = ref(3);

function applyIp() {
  if (ipInput.value.trim()) setIp(ipInput.value);
  else reconnect();
}

const fmt = (n: number) => (n > 0 ? String(n) : '-');

function drive(v: number, w: number) {
  outV.value = v;
  outW.value = w;
  setDrive(v, w);
}

function onJoy(v: number, w: number) { drive(v, w); }
function onJoyEnd() { outV.value = 0; outW.value = 0; stop(); }

// ---- 键盘 WASD / 方向键 ----
const keys = new Set<string>();
function applyKeys() {
  let v = 0, w = 0;
  if (keys.has('w') || keys.has('arrowup')) v += 70;
  if (keys.has('s') || keys.has('arrowdown')) v -= 70;
  if (keys.has('a') || keys.has('arrowleft')) w -= 60;
  if (keys.has('d') || keys.has('arrowright')) w += 60;
  if (v === 0 && w === 0) { outV.value = 0; outW.value = 0; stop(); }
  else drive(v, w);
}
function onKeyDown(e: KeyboardEvent) {
  const k = e.key.toLowerCase();
  if (!['w', 'a', 's', 'd', 'arrowup', 'arrowdown', 'arrowleft', 'arrowright', ' '].includes(k)) return;
  e.preventDefault();
  if (k === ' ') { stop(); return; }
  keys.add(k);
  applyKeys();
}
function onKeyUp(e: KeyboardEvent) {
  keys.delete(e.key.toLowerCase());
  applyKeys(); // 松开即按剩余按键重算，全松则归零（平滑由 20Hz+看门狗保证）
}

function teachBtn(_a: string) { /* 占位 */ }
function teach(action: string) { teachBtn(action); console.log('[teach] reserved:', action); }

onMounted(() => {
  window.addEventListener('keydown', onKeyDown);
  window.addEventListener('keyup', onKeyUp);
});
onBeforeUnmount(() => {
  window.removeEventListener('keydown', onKeyDown);
  window.removeEventListener('keyup', onKeyUp);
});
</script>

<style scoped>
.console { display: flex; flex-direction: column; height: 100%; padding: 12px; gap: 10px; max-width: 960px; margin: 0 auto; }
header { display: flex; justify-content: space-between; align-items: center; }
h1 { font-size: 18px; margin: 0; color: #58c4ff; }
.link { font-size: 13px; color: #ff5d5d; }
.link.on { color: #7dffb2; }
.ipbar { display: flex; gap: 8px; align-items: center; font-size: 13px; flex-wrap: wrap; }
.ipbar label { color: #7d93a8; }
.ipbar input { flex: 1; min-width: 130px; padding: 8px; border-radius: 8px; border: 1px solid #24405c; background: #060a10; color: #d7e3f0; }
.ipbar button { padding: 8px 14px; border-radius: 8px; border: 1px solid #24405c; background: #17324a; color: #fff; }
.ipbar button.ghost { background: transparent; }
.ipbar .url { color: #58c4ff; font-family: monospace; font-size: 12px; word-break: break-all; }
.err { color: #ff5d5d; font-size: 12px; margin: 0; }
.diags { border: 1px solid #24405c; border-radius: 10px; padding: 6px 10px; background: #060a10; }
.diags summary { cursor: pointer; font-size: 13px; color: #58c4ff; }
.diags pre { font-size: 11px; line-height: 1.5; color: #a9c3d8; white-space: pre-wrap; word-break: break-all; max-height: 180px; overflow-y: auto; margin: 6px 0 2px; }
main { display: grid; grid-template-columns: 1fr 1fr; gap: 10px; flex: 1; min-height: 0; }
.rangebar { display: flex; gap: 6px; align-items: center; font-size: 12px; color: #7d93a8; margin-bottom: 6px; }
.rangebar button { padding: 4px 12px; border-radius: 8px; border: 1px solid #24405c; background: #141d28; color: #d7e3f0; font-size: 12px; }
.rangebar button.on { background: #17324a; color: #fff; border-color: #58c4ff; }
@media (max-width: 720px) { main { grid-template-columns: 1fr; } }
.cards { display: grid; grid-template-columns: 1fr 1fr; gap: 8px; }
.card { background: #141d28; border: 1px solid #24405c; border-radius: 10px; padding: 8px 10px; }
.card small { color: #7d93a8; font-size: 11px; }
.card b { display: block; font-size: 17px; color: #7dffb2; }
.card b span { font-size: 11px; color: #7d93a8; }
.teach { display: flex; gap: 8px; margin-top: 10px; flex-wrap: wrap; }
.teach button { flex: 1; padding: 10px; border-radius: 10px; border: 1px solid #24405c; background: #141d28; color: #d7e3f0; }
.hint { font-size: 12px; color: #7d93a8; }
footer { display: flex; align-items: center; gap: 16px; }
.stop { padding: 16px 28px; border-radius: 12px; border: 1px solid #7a2b2b; background: #3a1414; color: #ffb3b3; font-size: 16px; }
.motor { border-top: 1px solid #24405c; padding-top: 10px; }
.motor h2 { font-size: 16px; margin: 0 0 8px; color: #58c4ff; }
.motor h2 small { font-size: 12px; color: #7d93a8; font-weight: normal; }
.motor h3 { font-size: 14px; margin: 0 0 6px; color: #d7e3f0; }
.motor .row { display: flex; gap: 6px; flex-wrap: wrap; margin: 6px 0; }
.motor .row button { flex: 1; min-width: 64px; padding: 9px; border-radius: 10px; border: 1px solid #24405c; background: #17324a; color: #fff; }
.motor .row button.warn { background: #3a1414; border-color: #7a2b2b; }
.mgrid { display: grid; grid-template-columns: 1fr 1fr; gap: 8px; }
@media (max-width: 720px) { .mgrid { grid-template-columns: 1fr; } }
.mcard { background: #141d28; border: 1px solid #24405c; border-radius: 10px; padding: 8px 10px; }
.mcard label { font-size: 12px; color: #7d93a8; }
.mcard label b { color: #7dffb2; }
.mcard input[type=range] { width: 100%; }
.mcard .tele { font-family: monospace; font-size: 12px; color: #7dffb2; }
</style>
