// 雷达 UDP→WebSocket 桥接（数据流 B 的 PC 侧转发层）。
//   ESP32 --UDP广播:8888--> 本脚本 --WS二进制:8889--> Vue 前端
// 中间层只做原样转发，绝不反序列化，零开销。
// 用法：npm run bridge   （需先 npm install 安装 ws）
// 前端 Radar 页连接 ws://localhost:8889 即可收到 254B ArrayBuffer 点云包。
const dgram = require('dgram');
const WebSocket = require('ws');

const UDP_PORT = Number(process.env.LIDAR_UDP_PORT ?? 8888);
const WS_PORT = Number(process.env.LIDAR_WS_PORT ?? 8889);

const udp = dgram.createSocket('udp4');
const wss = new WebSocket.Server({ port: WS_PORT });

let clients = 0;

wss.on('connection', (ws) => {
  clients++;
  console.log(`[Bridge] 前端已连接 ws://localhost:${WS_PORT}，在线=${clients}`);
  ws.on('close', () => {
    clients--;
    console.log(`[Bridge] 前端断开，剩余在线=${clients}`);
  });
  ws.on('error', (e) => console.warn(`[Bridge] WS 错误: ${e.message}`));
});

udp.on('error', (err) => {
  console.error(`[Bridge] UDP 异常:\n${err.stack}`);
  try { udp.close(); } catch { /* ignore */ }
  process.exit(1);
});

udp.on('message', (msg) => {
  // 254B 二进制包：magic(0xAA55) 校验，不合法直接丢弃
  if (msg.length < 14 || msg.readUInt16LE(0) !== 0xaa55) return;
  if (clients === 0) return; // 无前端时直接丢弃，不积压
  for (const c of wss.clients) {
    if (c.readyState === WebSocket.OPEN) c.send(msg);
  }
});

udp.bind(UDP_PORT, () => {
  console.log(`[Bridge] UDP 监听 0.0.0.0:${UDP_PORT}，二进制 WS 桥接 ws://localhost:${WS_PORT}`);
  try { udp.addMembership('255.255.255.255'); } catch { /* 广播无需组播 */ }
});
