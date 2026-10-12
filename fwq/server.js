// fwq/server.js —— 零依赖静态托管 + UDP 雷达桥接（需 npm install 安装 ws）
// 用法：npm install && node server.js [端口，默认 8080]
//   HTTP  : http://localhost:8080/          静态页面
//   UDP   : 0.0.0.0:8888                    收 ESP32 254B 雷达广播
//   WS    : ws://localhost:8889            给前端下发 JSON 点云 {rpm,seq,pts:[[angle,dist]..]}
const http = require('http');
const fs = require('fs');
const path = require('path');
const dgram = require('dgram');

const PORT = Number(process.argv[2] || 8080);
const UDP_PORT = 8888;
const WS_PORT = 8889;
const ROOT = __dirname;
const TYPES = {
  '.html': 'text/html; charset=utf-8',
  '.js': 'application/javascript; charset=utf-8',
  '.css': 'text/css; charset=utf-8',
  '.json': 'application/json; charset=utf-8',
};

http.createServer((req, res) => {
  let p = decodeURIComponent(req.url.split('?')[0]);
  if (p === '/') p = '/index.html';
  const file = path.normalize(path.join(ROOT, p));
  if (!file.startsWith(ROOT)) {
    res.writeHead(403); res.end('forbidden'); return;
  }
  fs.readFile(file, (err, data) => {
    if (err) {
      res.writeHead(404); res.end('not found: ' + p); return;
    }
    res.writeHead(200, { 'Content-Type': TYPES[path.extname(file)] || 'application/octet-stream' });
    res.end(data);
  });
}).listen(PORT, () => console.log(`[fwq] http://localhost:${PORT}/`));

// ---- UDP 雷达桥接：254B 二进制 → JSON 点云 → WS 广播 ----
// 包结构（LE）：magic u16(0xAA55) | seq u16 | rpm u16 | front/left/right u16 |
//              count u16 | distances[120] u16（索引 i ↔ i*3°）
let wss = null;
try {
  const WebSocket = require('ws');
  wss = new WebSocket.Server({ port: WS_PORT });
  wss.on('connection', (ws) => {
    ws.on('error', () => {});
  });
  console.log(`[fwq] 雷达 WS 桥接 ws://localhost:${WS_PORT}`);
} catch (e) {
  console.log('[fwq] 未找到 ws 模块，雷达桥接禁用：请执行 npm install');
}

const udp = dgram.createSocket('udp4');
udp.on('error', (err) => {
  console.error(`[fwq] UDP 异常: ${err.message}`);
});
udp.on('message', (msg) => {
  if (msg.length !== 254 || msg.readUInt16LE(0) !== 0xaa55) return;
  if (!wss || wss.clients.size === 0) return; // 无前端时直接丢弃，不积压
  const seq = msg.readUInt16LE(2);
  const rpm = msg.readUInt16LE(4);
  const count = msg.readUInt16LE(12);
  const pts = [];
  for (let i = 0; i < count && i < 120; i++) {
    const d = msg.readUInt16LE(14 + i * 2);
    if (d > 0) pts.push([i * 3, d]); // [角度°, 距离mm]，只发有效点
  }
  const out = JSON.stringify({ t: 'lidar', seq, rpm, n: pts.length, pts });
  for (const c of wss.clients) {
    if (c.readyState === 1) c.send(out);
  }
});
udp.bind(UDP_PORT, () => console.log(`[fwq] UDP 监听 0.0.0.0:${UDP_PORT}`));
