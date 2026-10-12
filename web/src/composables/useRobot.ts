import { reactive, ref, computed, onUnmounted } from 'vue';

export interface LidarState {
  ok: boolean;
  rpm: number;
  front: number;
  left: number;
  right: number;
  gmin: number;
  gangle: number;
  escape: number;
  pts: number[]; // 120 点，每 3° 一点，单位 mm
  stamp: number; // 末帧解包的 Date.now()，供渲染循环与诊断使用
}

const empty: LidarState = {
  ok: false, rpm: 0, front: 0, left: 0, right: 0,
  gmin: 0, gangle: 0, escape: 0, pts: [], stamp: 0,
};

const DEFAULT_IP = '192.168.137.148';
const LS_KEY = 'robot_ip';
const RETRY_MS = 4000; // 失联后 4s 重试，避免高频轰炸板子

// 数据流 B：本地 UDP→WS 桥接（npm run bridge），二进制 254B 点云包
const BRIDGE_URL = 'ws://localhost:8889';
const UDP_MAGIC = 0xaa55;
const UDP_POINTS = 120;

// 与 ESP32 /ws/lidar 的唯一长连接：雷达下行 + 遥控上行复用。
// 地址策略（默认相对路径 → dev 下走 Vite 同源代理，生产走页面 Host）：
//   ws(s)://location.host/ws/lidar，除非用户在设置栏指定了目标 IP。
// 模块级单例锁：HMR/重挂载时旧 socket 若还在 CONNECTING/OPEN，直接复用。
let sharedWs: WebSocket | null = null;
let ws: WebSocket | null = null;

// ---- 雷达字段兼容适配器 ----
// 后端键名可能漂移（pts/points/data，front/front_dist，rpm/speed），
// 点云元素可能为 number(mm) / [angle,dist] 二元组 / {a,d} 对象，统一归一为 number[](mm)。
function toMillimeters(v: unknown): number {
  const n = typeof v === 'number' ? v : Number(v);
  if (!Number.isFinite(n) || n < 0) return 0;
  return Math.round(n);
}

function normalizePts(src: unknown): number[] {
  if (!Array.isArray(src)) return [];
  const out: number[] = new Array(src.length);
  for (let i = 0; i < src.length; i++) {
    const e = src[i] as unknown;
    if (typeof e === 'number' || typeof e === 'string') {
      out[i] = toMillimeters(e);
    } else if (Array.isArray(e)) {
      // [angle, distance] 取第二元；若只有一元则取首元
      out[i] = toMillimeters(e.length >= 2 ? e[1] : e[0]);
    } else if (e !== null && typeof e === 'object') {
      const o = e as Record<string, unknown>;
      const cand = o.d ?? o.dist ?? o.distance ?? o.r ?? o.range ?? o.v ?? o.value;
      out[i] = toMillimeters(cand);
    } else {
      out[i] = 0;
    }
  }
  return out;
}

// eslint-disable-next-line @typescript-eslint/no-explicit-any
function adaptLidar(json: any): LidarState {
  const rawPts = json.pts ?? json.points ?? json.data ?? [];
  return {
    ok: Boolean(json.ok ?? true),
    rpm: toMillimeters(json.rpm ?? json.speed ?? 0),
    front: toMillimeters(json.front ?? json.front_dist ?? 0),
    left: toMillimeters(json.left ?? json.left_dist ?? 0),
    right: toMillimeters(json.right ?? json.right_dist ?? 0),
    gmin: toMillimeters(json.gmin ?? json.min_dist ?? 0),
    gangle: toMillimeters(json.gangle ?? json.min_angle ?? 0),
    escape: toMillimeters(json.escape ?? json.heading ?? 0),
    pts: normalizePts(rawPts),
    stamp: Date.now(),
  };
}

export function useRobot() {
  return sharedStore();
}

// ---- 模块级单例状态：HMR/重挂载时复用既有连接必须写同一份 lidar，
// 否则“复用既有连接”会把帧写进旧实例，新页面永远 0 点 ----
const lidar = reactive<LidarState>({ ...empty, pts: [] });
const connected = ref(false);
const rttMs = ref(-1);
const lastUrl = ref('');
const lastError = ref('');
const targetIp = ref(localStorage.getItem(LS_KEY) ?? DEFAULT_IP);
const useCustomIp = ref(localStorage.getItem(LS_KEY) !== null);
const logs = ref<string[]>([]);
const rxPerSec = ref(0);
let storeInit = false;
let pumpTimer: number | null = null;
let retryTimer: number | null = null;
let connectTimer: number | null = null;
const CONNECT_TIMEOUT_MS = 6000;
let closedByUser = false;
let pending: { v: number; w: number } | null = null;
let stopSent = true; // 静默保护：静止时只发一帧停止，之后静默
let rxCount = 0;
let rxWindowAt = performance.now();
let msgSeq = 0;
let noMsgTimer: number | null = null;
// 数据流 B（桥接二进制）状态
let bridgeWs: WebSocket | null = null;
let bridgeTimer: number | null = null;
let bridgeSeq = 0;
let lastBridgeSeq = -1;
const bridgeOn = ref(false);
const udpPerSec = ref(0);

function sharedStore() {

  function log(msg: string) {
    const t = new Date();
    const ts = [t.getHours(), t.getMinutes(), t.getSeconds()]
      .map((n) => String(n).padStart(2, '0')).join(':');
    const line = `[${ts}] ${msg}`;
    console.log('[robot]', line);
    logs.value.push(line);
    if (logs.value.length > 80) logs.value.splice(0, logs.value.length - 80);
  }

  const wsUrl = computed(() => {
    if (useCustomIp.value) {
      return `ws://${targetIp.value}/ws/lidar`; // 设置栏指定：直连设备
    }
    // 默认相对路径：dev 经 Vite 代理同源转发，生产跟页面 Host。
    const proto = location.protocol === 'https:' ? 'wss://' : 'ws://';
    return `${proto}${location.host}/ws/lidar`;
  });

  // ---- 数据流 B：桥接二进制点云（ArrayBuffer/DataView 解包） ----
  // 包结构（LE）：magic u16 | seq u16 | rpm u16 | front/left/right u16 |
  //              count u16 | distances[120] u16，共 254B
  function parseUdpPacket(buf: ArrayBuffer): boolean {
    // 严格对齐 254B 协议（LE）：magic(0-1) seq(2-3) rpm(4-5) front(6-7)
    // left(8-9) right(10-11) count(12-13) distances[120](14-253)
    if (buf.byteLength !== 14 + UDP_POINTS * 2) {
      if (bridgeSeq === 0) {
        log(`UDP 首包长度异常 ${buf.byteLength}B（期望 254），检查桥接/固件版本`);
      }
      return false;
    }
    const dv = new DataView(buf);
    if (dv.getUint16(0, true) !== UDP_MAGIC) return false;
    const seq = dv.getUint16(2, true);
    if (seq === lastBridgeSeq) return false; // 去重（桥接重发保护，u16 回绕不影响）
    const count = dv.getUint16(12, true);
    if (count !== UDP_POINTS) return false;
    lastBridgeSeq = seq;
    lidar.ok = true;
    lidar.rpm = dv.getUint16(4, true);
    lidar.front = dv.getUint16(6, true);
    lidar.left = dv.getUint16(8, true);
    lidar.right = dv.getUint16(10, true);
    // 整体替换 pts 引用：shallow watch 必触发；timestamp 供调试
    const pts = new Array<number>(UDP_POINTS);
    for (let i = 0; i < UDP_POINTS; i++) {
      pts[i] = dv.getUint16(14 + i * 2, true);
    }
    lidar.pts = pts;
    lidar.stamp = Date.now();
    bridgeSeq++;
    rxCount++;
    return true;
  }

  function connectBridge() {
    if (bridgeWs && (bridgeWs.readyState === 0 || bridgeWs.readyState === 1)) return;
    try {
      bridgeWs = new WebSocket(BRIDGE_URL);
    } catch {
      scheduleBridgeRetry();
      return;
    }
    bridgeWs.binaryType = 'arraybuffer';
    bridgeWs.onopen = () => {
      bridgeOn.value = true;
      log(`雷达桥接已连接 ${BRIDGE_URL}（UDP 二进制流）`);
    };
    bridgeWs.onmessage = (e: MessageEvent) => {
      const data = e.data;
      if (data instanceof ArrayBuffer) {
        parseUdpPacket(data);
      } else if (data instanceof Blob) {
        void data.arrayBuffer().then((b) => parseUdpPacket(b));
      }
    };
    bridgeWs.onclose = () => {
      bridgeOn.value = false;
      bridgeWs = null;
      scheduleBridgeRetry();
    };
    bridgeWs.onerror = () => {
      // 桥接未启动（npm run bridge）时静默重试，不刷屏
      try { bridgeWs?.close(); } catch { /* ignore */ }
    };
  }

  function scheduleBridgeRetry() {
    if (closedByUser || bridgeTimer !== null) return;
    // 桥接未启动时浏览器控制台会有一次失败打印，退避到 8s 避免刷屏
    bridgeTimer = window.setTimeout(() => {
      bridgeTimer = null;
      connectBridge();
    }, 8000);
  }

  // ---- 数据流 A：状态 REST 轮询（0.5Hz，补 gmin/gangle/escape 等二进制包外字段） ----
  // 高速特征（rpm/front/left/right）由 UDP 二进制 10Hz 覆盖，此处只补低速字段，
  // 频率必须低，否则短连接堆积会挤爆 ESP32 HTTPD（accept ENFILE）。
  // DEV 下一律走 Vite 同源代理，彻底规避 CORS；生产直连设备 IP（已加 CORS 头兜底）。
  let statusTimer: number | null = null;
  function statusUrl(): string {
    if (import.meta.env.DEV) return '/api/lidar_status';
    if (useCustomIp.value) return `http://${targetIp.value}/api/lidar_status`;
    return '/api/lidar_status';
  }
  function pollStatus() {
    fetch(statusUrl())
      .then((r) => (r.ok ? r.json() : null))
      .then((j) => {
        if (!j) return;
        lidar.gmin = toMillimeters(j.gmin ?? 0);
        lidar.gangle = toMillimeters(j.gangle ?? 0);
        lidar.escape = toMillimeters(j.escape ?? 0);
        // 桥接未连时，REST 兼带基础特征，避免卡片全空
        if (!bridgeOn.value && j.ok) {
          lidar.ok = true;
          lidar.rpm = toMillimeters(j.rpm ?? lidar.rpm);
          lidar.front = toMillimeters(j.front ?? lidar.front);
          lidar.left = toMillimeters(j.left ?? lidar.left);
          lidar.right = toMillimeters(j.right ?? lidar.right);
        }
      })
      .catch(() => { /* 设备离线时静默 */ });
  }

  // ---- 数据流 A：点云 HTTP 降级（桥接断开时 1Hz，桥接正常时零调用） ----
  // 无论点云来自桥接二进制还是本降级通道，最终都写入同一 lidar.pts，
  // Canvas 只认 pts，彻底消除“两张皮”。
  function pointsUrl(): string {
    if (import.meta.env.DEV) return '/api/lidar_points';
    if (useCustomIp.value) return `http://${targetIp.value}/api/lidar_points`;
    return '/api/lidar_points';
  }
  function pollPoints() {
    if (bridgeOn.value) return; // 桥接正常：10Hz 二进制优先，本通道静默
    fetch(pointsUrl())
      .then((r) => (r.ok ? r.json() : null))
      .then((j) => {
        if (!j || bridgeOn.value) return;
        const pts = normalizePts(j.pts ?? j.points ?? j.data ?? []);
        if (pts.length === 0) return;
        lidar.ok = true;
        lidar.pts = pts;
        lidar.stamp = Date.now();
        rxCount++;
      })
      .catch(() => { /* 设备离线时静默 */ });
  }

  function scheduleRetry() {
    if (closedByUser || retryTimer !== null) return;
    log(`${RETRY_MS / 1000}s 后重试…`);
    retryTimer = window.setTimeout(() => {
      retryTimer = null;
      connect();
    }, RETRY_MS);
  }

  function connect(force = false) {
    // 单例锁：非强制时旧 socket 还在 CONNECTING/OPEN → 复用，严禁并发建连。
    // 强制重连（用户点“重新连接”/换IP）则直接丢弃旧 socket 新建，避免卡在 readyState=0。
    if (!force && sharedWs && (sharedWs.readyState === 0 || sharedWs.readyState === 1)) {
      log(`复用既有连接（readyState=${sharedWs.readyState}），跳过新建`);
      ws = sharedWs;
      connected.value = sharedWs.readyState === 1;
      return;
    }
    cleanupSocket();
    closedByUser = false;
    const url = wsUrl.value;
    lastUrl.value = url;
    lastError.value = '';
    log(`尝试连接 ${url}（DEV=${import.meta.env.DEV ? 'yes' : 'no'}，自定义IP=${useCustomIp.value ? 'yes' : 'no'}）`);
    try {
      ws = new WebSocket(url);
    } catch (e) {
      const m = `构造 WebSocket 异常: ${String(e)}`;
      lastError.value = m;
      log(m);
      scheduleRetry();
      return;
    }
    ws.onopen = () => {
      if (connectTimer !== null) { clearTimeout(connectTimer); connectTimer = null; }
      connected.value = true;
      lastError.value = '';
      rxCount = 0;
      rxWindowAt = performance.now();
      msgSeq = 0;
      log(`已连接 ${url}`);
      // 看门狗：控制连上 3s 且雷达依然 0 帧 → 提示查桥接/UDP
      if (noMsgTimer !== null) clearTimeout(noMsgTimer);
      noMsgTimer = window.setTimeout(() => {
        noMsgTimer = null;
        if (bridgeSeq === 0 && msgSeq === 0 && ws && ws.readyState === 1) {
          log(`雷达 0 帧：检查 npm run bridge 是否启动`);
        }
      }, 3000);
    };
    ws.onmessage = (e: MessageEvent) => {
      // 数据流 A 控制链路下行：新固件无推送，此处仅兼容旧固件 JSON 推送。
      // 主雷达流走桥接二进制（parseUdpPacket），两者互补。
      try {
        const raw = typeof e.data === 'string' ? e.data : null;
        if (raw === null) return; // 非文本帧忽略
        msgSeq++;
        if (noMsgTimer !== null) { clearTimeout(noMsgTimer); noMsgTimer = null; }
        if (raw.length < 10 || raw.length > 8000) return;
        const adapted = adaptLidar(JSON.parse(raw));
        // 整体替换 pts 引用，确保 RadarCanvas 的浅 watch 必触发
        lidar.ok = adapted.ok;
        lidar.rpm = adapted.rpm;
        lidar.front = adapted.front;
        lidar.left = adapted.left;
        lidar.right = adapted.right;
        lidar.gmin = adapted.gmin;
        lidar.gangle = adapted.gangle;
        lidar.escape = adapted.escape;
        lidar.pts = adapted.pts;
      } catch {
        // 旧固件 JSON 解析失败则静默丢弃
      }
    };
    ws.onerror = () => {
      lastError.value = `连接失败 ${url}`;
      log(`连接异常（握手失败/拒连/设备离线），${RETRY_MS / 1000}s 后重试`);
    };
    ws.onclose = (e: CloseEvent) => {
      if (connectTimer !== null) { clearTimeout(connectTimer); connectTimer = null; }
      if (noMsgTimer !== null) { clearTimeout(noMsgTimer); noMsgTimer = null; }
      connected.value = false;
      if (sharedWs === ws) sharedWs = null;
      lastError.value = `已断开 ${url}（code=${e.code} reason=${e.reason || '无'}）`;
      log(`onclose code=${e.code} reason=${e.reason || '无'} wasClean=${e.wasClean}`);
      scheduleRetry();
    };
    sharedWs = ws;
    // 建连超时保护：CONNECTING 卡住（如目标IP离线，浏览器TCP超时很长）则主动掐断，触发 onclose→重试。
    if (connectTimer !== null) clearTimeout(connectTimer);
    connectTimer = window.setTimeout(() => {
      connectTimer = null;
      if (ws && ws.readyState === 0) {
        log(`建连超时（${CONNECT_TIMEOUT_MS}ms 仍 CONNECTING），主动关闭: ${url}`);
        try { ws.close(); } catch { /* ignore */ }
      }
    }, CONNECT_TIMEOUT_MS);
  }

  function cleanupSocket() {
    if (retryTimer !== null) {
      clearTimeout(retryTimer);
      retryTimer = null;
    }
    if (connectTimer !== null) {
      clearTimeout(connectTimer);
      connectTimer = null;
    }
    if (ws) {
      ws.onopen = ws.onmessage = ws.onerror = ws.onclose = null;
      try { ws.close(); } catch { /* ignore */ }
      if (sharedWs === ws) sharedWs = null;
      ws = null;
    }
  }

  function setIp(ip: string) {
    targetIp.value = ip.trim();
    useCustomIp.value = true;
    localStorage.setItem(LS_KEY, targetIp.value);
    reconnect();
  }

  function useAutoIp() {
    useCustomIp.value = false;
    localStorage.removeItem(LS_KEY);
    reconnect();
  }

  function reconnect() {
    connected.value = false;
    if (retryTimer !== null) { clearTimeout(retryTimer); retryTimer = null; }
    connect(true);
  }

  // 5Hz 节流发送（200ms 一帧），静止时静默，禁止刷包打爆 lwIP。
  function pump() {
    if (!ws || ws.readyState !== 1) return;
    if (!pending) return;
    const p = pending;
    const isStop = Math.round(p.v) === 0 && Math.round(p.w) === 0;
    if (isStop) {
      if (stopSent) { pending = null; return; }
      stopSent = true;
    } else {
      stopSent = false;
    }
    ws.send(JSON.stringify({ cmd: 'drive', v: Math.round(p.v), w: Math.round(p.w) }));
    pending = isStop ? null : pending;
  }

  function setDrive(v: number, w: number) {
    pending = { v, w };
    if (v !== 0 || w !== 0) stopSent = false;
  }

  function stop() {
    if (stopSent && !pending) return;
    pending = { v: 0, w: 0 };
    if (ws && ws.readyState === 1) {
      ws.send(JSON.stringify({ cmd: 'drive', v: 0, w: 0 }));
      pending = null;
      stopSent = true;
    }
  }

  connect();
  connectBridge(); // 数据流 B：桥接二进制（失败则 4s 重试，不影响控制链路）
  if (!storeInit) {
    storeInit = true;
    pumpTimer = window.setInterval(pump, 200); // 数据流 A 上行 5Hz
    statusTimer = window.setInterval(pollStatus, 2000); // 数据流 A 状态 0.5Hz
    window.setInterval(pollPoints, 500); // 数据流 A 点云降级 2Hz（桥接正常时内部直接返回）
    window.setInterval(() => {
      const now = performance.now();
      const dt = (now - rxWindowAt) / 1000;
      udpPerSec.value = dt > 0 ? Math.round(rxCount / dt) : 0;
      rxCount = 0;
      rxWindowAt = now;
    }, 1000);
  }
  onUnmounted(() => {
    // 单例 store：组件卸载只停自己的 pump 标记，不断 sharedWs，
    // 避免新实例“复用既有连接”却收不到帧。
    closedByUser = false;
  });

  // HMR 热更新时释放旧连接，避免 dev 下 socket 越积越多。
  if (import.meta.hot) {
    import.meta.hot.dispose(() => {
      closedByUser = true;
      if (pumpTimer) clearInterval(pumpTimer);
      if (statusTimer) clearInterval(statusTimer);
      if (bridgeTimer) clearTimeout(bridgeTimer);
      try { bridgeWs?.close(); } catch { /* ignore */ }
      bridgeWs = null;
      cleanupSocket();
    });
  }

  log(`启动: DEV=${import.meta.env.DEV ? 'yes' : 'no'}，目标IP=${targetIp.value}，页面host=${location.host}`);

  return {
    lidar, connected, rttMs, lastUrl, lastError, logs, rxPerSec,
    targetIp, useCustomIp, wsUrl, bridgeOn, udpPerSec,
    setDrive, stop, setIp, useAutoIp, reconnect,
  };
}
