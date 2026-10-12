/* fwq/app.js —— MQTT over WebSocket 双轮底盘控制台核心逻辑
 * 依赖：mqtt.min.js（CDN，index.html 引入）
 * 协议：robot/cmd 下发 {action,speed_l,speed_r} QoS0；robot/telemetry 订阅 QoS0
 * 安全：200ms 心跳持续下发当前速度（兼 2s 看门狗保活）；松手/断键即发 stop
 */
(function () {
  'use strict';

  var LS_IP = 'fwq_broker_ip';
  var LS_PORT = 'fwq_broker_port';
  var DEF_IP = '103.231.59.46';
  var DEF_PORT = '8083';

  var client = null;
  var connected = false;
  // 当前运动意图：action + 双轮速度；心跳每 200ms 原样下发。
  // manual 模式下意图即 man 双轮值（滑块/单轮按钮持久保持），心跳不会将其清零。
  var intent = { action: 'stop', speed_l: 0, speed_r: 0 };
  var man = { l: 0, r: 0 }; // 独立调试双轮保持值
  var cruise = 60; // 十字键档位速度
  var beatAt = 0;

  var $ = function (id) { return document.getElementById(id); };

  // ---------- 连接管理 ----------
  function setConn(state, text) {
    var dot = $('dot');
    dot.className = 'dot' + (state === 'on' ? ' on' : state === 'mid' ? ' mid' : '');
    $('connText').textContent = text;
  }

  function toggle() {
    if (connected && client) { client.end(); return; }
    var ip = $('brokerIp').value.trim() || DEF_IP;
    var port = $('brokerPort').value.trim() || DEF_PORT;
    localStorage.setItem(LS_IP, ip);
    localStorage.setItem(LS_PORT, port);
    var url = 'ws://' + ip + ':' + port + '/mqtt';
    setConn('mid', '连接中 ' + url + ' …');
    client = mqtt.connect(url, { connectTimeout: 5000, reconnectPeriod: 3000 });
    client.on('connect', function () {
      connected = true;
      $('connBtn').textContent = '断开';
      setConn('on', '在线 @5Hz 心跳');
      client.subscribe('robot/telemetry', { qos: 0 });
    });
    client.on('message', function (topic, msg) {
      if (topic !== 'robot/telemetry') return;
      try {
        var j = JSON.parse(msg.toString());
        $('rpmA').textContent = j.rpm_l;
        $('cntA').textContent = j.cnt_l;
        $('rpmB').textContent = j.rpm_r;
        $('cntB').textContent = j.cnt_r;
        beatAt = Date.now();
        $('beatText').textContent = '心跳正常 · ' + new Date(beatAt).toLocaleTimeString();
      } catch (e) { /* 坏帧丢弃 */ }
    });
    client.on('close', function () {
      connected = false;
      $('connBtn').textContent = '连接';
      setConn('', '离线（重连中…）');
    });
    client.on('error', function () {
      setConn('', '连接错误：检查 IP/端口与 EMQX 8083 放行');
    });
  }

  // ---------- 下发 ----------
  function pub(payload) {
    if (!connected || !client) return;
    client.publish('robot/cmd', JSON.stringify(payload), { qos: 0 });
  }
  // 200ms 心跳：下发当前意图（松手后意图为 stop，车自动停；也喂饱 2s 看门狗）
  setInterval(function () {
    if (!connected) return;
    pub({ action: intent.action, speed_l: intent.speed_l, speed_r: intent.speed_r });
  }, 200);

  function drive(act) {
    var s = cruise;
    var m = { action: act, speed_l: 0, speed_r: 0 };
    if (act === 'forward') { m.speed_l = s; m.speed_r = s; }
    else if (act === 'backward') { m.speed_l = -s; m.speed_r = -s; }
    else if (act === 'left') { m.speed_l = -s; m.speed_r = s; }
    else if (act === 'right') { m.speed_l = s; m.speed_r = -s; }
    intent = m;
    pub(m); // 按下即时下发，不等心跳
  }
  function halt(brake) {
    intent = { action: brake ? 'brake' : 'stop', speed_l: 0, speed_r: 0 };
    pub(intent);
  }

  // ---------- 十字键：触屏 + 鼠标点动/长按 ----------
  document.querySelectorAll('button.hold').forEach(function (b) {
    var act = b.getAttribute('data-act');
    var on = function (e) {
      e.preventDefault();
      b.classList.add('on');
      drive(act);
    };
    var off = function () {
      if (!b.classList.contains('on')) return;
      b.classList.remove('on');
      halt(false);
    };
    b.addEventListener('touchstart', on, { passive: false });
    b.addEventListener('touchend', off);
    b.addEventListener('touchcancel', off);
    b.addEventListener('mousedown', on);
    b.addEventListener('mouseup', off);
    b.addEventListener('mouseleave', off);
  });
  $('brakeBtn').addEventListener('click', function () { halt(true); });

  // ---------- 键盘 ----------
  var keyAct = { KeyW: 'forward', ArrowUp: 'forward', KeyS: 'backward', ArrowDown: 'backward',
                 KeyA: 'left', ArrowLeft: 'left', KeyD: 'right', ArrowRight: 'right' };
  var keyDown = null;
  document.addEventListener('keydown', function (e) {
    if (e.repeat) return;
    if (e.code === 'Space') { e.preventDefault(); halt(true); return; }
    var act = keyAct[e.code];
    if (!act) return;
    e.preventDefault();
    keyDown = act;
    document.querySelectorAll('button.hold').forEach(function (b) {
      b.classList.toggle('on', b.getAttribute('data-act') === act);
    });
    drive(act);
  });
  document.addEventListener('keyup', function (e) {
    var act = keyAct[e.code];
    if (!act || keyDown !== act) return;
    keyDown = null;
    document.querySelectorAll('button.hold').forEach(function (b) { b.classList.remove('on'); });
    halt(false);
  });

  // ---------- 档位与独立调试 ----------
  $('spd').addEventListener('input', function () {
    cruise = +$('spd').value;
    $('spdVal').textContent = cruise;
  });
  function bindMotor(sliderId, valId, motor) {
    $(sliderId).addEventListener('input', function () {
      var v = +$(sliderId).value;
      $(valId).textContent = v;
      if (motor === 'A') man.l = v; else man.r = v;
      intent = { action: 'manual', speed_l: man.l, speed_r: man.r };
      pub(intent);
    });
  }
  bindMotor('sA', 'vA', 'A');
  bindMotor('sB', 'vB', 'B');
  document.querySelectorAll('[data-ma]').forEach(function (b) {
    b.addEventListener('click', function () {
      var motor = b.getAttribute('data-ma');
      var dir = +b.getAttribute('data-md'); // 1正转 -1反转 0停
      var sliderId = motor === 'A' ? 'sA' : 'sB';
      var valId = motor === 'A' ? 'vA' : 'vB';
      var v = dir === 0 ? 0 : (+$(sliderId).value || 60) * dir;
      if (dir !== 0 && +$(sliderId).value === 0) {
        v = 60 * dir;
      }
      $(sliderId).value = v;
      $(valId).textContent = v;
      if (motor === 'A') man.l = v; else man.r = v;
      intent = { action: 'manual', speed_l: man.l, speed_r: man.r };
      pub(intent);
    });
  });

  // ---------- 激光雷达点云（WS :8889 JSON + rAF 荧光余晖） ----------
  // 网络只写后台缓冲，渲染独立 60fps：MQTT 指令与点云互不阻塞
  var lidarPts = []; // [[angle°, distMm], ...] 后台缓冲
  var lidarRpm = 0;
  var lidarSeq = -1;
  var lidarAt = 0;
  var lidarRange = 3; // 量程半径（m）：推杆连续 0.5~6，默认 3m
  var Z_MIN = 0.5, Z_DEF = 3, Z_MAX = 6;

  // 固定刻度环：像素位置永不动（1/6、1/3、2/3、满半径），只重算标签数字。
  // 标签 = 环半径占比 × 当前量程，保留两位小数去尾零。
  var RING_FRACS = [1 / 6, 1 / 3, 2 / 3, 1];
  function fmtM(v) {
    return (Math.round(v * 100) / 100) + 'm';
  }
  function rangeRings() {
    return RING_FRACS.map(function (f) { return [f * lidarRange, fmtM(f * lidarRange)]; });
  }

  function fmtRange(v) {
    return (Math.round(v * 100) / 100) + 'm';
  }

  function initLidar() {
    var cv = $('lidar');
    fitLidar(cv);
    window.addEventListener('resize', function () { fitLidar(cv); });
    var ws;
    try {
      ws = new WebSocket('ws://' + location.hostname + ':8889');
    } catch (e) {
      $('lstatText').textContent = '雷达 WS 构造失败';
      requestAnimationFrame(function () { drawLidar(cv); });
      return;
    }
    ws.onopen = function () { $('lstatText').textContent = '雷达链路已连接 @20Hz'; };
    ws.onclose = function () {
      $('lstatText').textContent = '雷达链路断开（server.js 是否在跑？）';
      setTimeout(initLidar, 3000);
    };
    ws.onerror = function () { /* 出错即走 onclose 重连，不刷屏 */ };
    ws.onmessage = function (ev) {
      try {
        var j = JSON.parse(ev.data);
        if (j.t !== 'lidar' || !j.pts) return;
        if (j.seq === lidarSeq) return; // 去重
        lidarSeq = j.seq;
        lidarRpm = j.rpm;
        lidarPts = j.pts; // 整体替换，后台缓冲无 DOM 操作
        lidarAt = Date.now();
        $('lrpm').textContent = j.rpm;
        $('ln').textContent = j.n + '/120';
      } catch (e) { /* 坏帧丢弃 */ }
    };
    requestAnimationFrame(function () { drawLidar(cv); });
  }

  function fitLidar(cv) {
    // 盒子 aspect-ratio 定宽高：取可视宽高中较小者为正方形边，
    // 与 CSS width/height:100% 对齐，杜绝圆盘拉成椭圆
    var box = cv.parentElement;
    var side = Math.min(box.clientWidth || 0, box.clientHeight || 0);
    if (!side || side <= 0) side = Math.min(box.clientWidth || 360, 560);
    side = Math.min(side, 560);
    var dpr = Math.min(window.devicePixelRatio || 1, 2);
    cv.width = Math.round(side * dpr);
    cv.height = Math.round(side * dpr);
  }

  // 浅色盘配色（与 #f8fafc 盒底一致）：深色网格字、实色散点（浅底不用 lighter 叠加）
  var L_BG = '#f8fafc';
  var L_FADE = 'rgba(248,250,252,0.25)';
  var L_GRID = '#c3cedb';
  var L_TEXT = '#64748b';
  var L_SELF = '#0284c7';

  function drawLidar(cv) {
    var ctx = cv.getContext('2d');
    var dpr = Math.min(window.devicePixelRatio || 1, 2);
    var size = cv.width / dpr;
    if (size > 0) {
      ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
      var C = size / 2;
      var S = C / lidarRange; // 满量程打满半径
      // 余晖：半透明浅底覆盖上一帧（首帧铺实底防透明叠加）
      ctx.fillStyle = lidarSeq < 0 ? L_BG : L_FADE;
      ctx.fillRect(0, 0, size, size);
      // 极坐标网格（每帧重绘保持锐利，开销可忽略）
      ctx.strokeStyle = L_GRID;
      ctx.fillStyle = L_TEXT;
      ctx.font = '11px sans-serif';
      ctx.textAlign = 'left';
      rangeRings().forEach(function (r) {
        ctx.beginPath();
        ctx.arc(C, C, r[0] * S, 0, Math.PI * 2);
        ctx.stroke();
        ctx.fillText(r[1], C + 4, C - r[0] * S - 4);
      });
      // 本体小箭头（正上方 = 0°）
      ctx.strokeStyle = L_SELF;
      ctx.fillStyle = L_SELF;
      ctx.beginPath(); ctx.moveTo(C, C); ctx.lineTo(C, 10); ctx.stroke();
      ctx.beginPath();
      ctx.moveTo(C, 6); ctx.lineTo(C - 5, 16); ctx.lineTo(C + 5, 16); ctx.fill();
      // 实色散点（浅底不做 lighter 叠加，否则泛白）
      var maxMm = lidarRange * 1000;
      for (var i = 0; i < lidarPts.length; i++) {
        var a = lidarPts[i][0] * Math.PI / 180;
        var r = Math.min(lidarPts[i][1], maxMm) / 1000 * S;
        var x = C + r * Math.sin(a);
        var y = C - r * Math.cos(a);
        var d = lidarPts[i][1];
        ctx.fillStyle = d < 400 ? '#dc2626' : d < 1200 ? '#d97706' : '#059669';
        var s = d < 400 ? 2 : 1;
        ctx.fillRect(x - s / 2, y - s / 2, s, s);
      }
      if (Date.now() - lidarAt > 1500) {
        $('lstatText').textContent = '点云超时（UDP :8888 是否到达本机？）';
      }
    }
    requestAnimationFrame(function () { drawLidar(cv); });
  }
  $('brokerIp').value = localStorage.getItem(LS_IP) || DEF_IP;
  $('brokerPort').value = localStorage.getItem(LS_PORT) || DEF_PORT;
  $('connBtn').addEventListener('click', toggle);
  // 量程切换：只改本地显示比例，不动任何数据链路
  function markRangeButtons() {
    document.querySelectorAll('#rangeBar .range-btn').forEach(function (x) {
      x.classList.toggle('on', Math.abs(+x.getAttribute('data-range') - lidarRange) < 1e-6);
    });
  }
  document.querySelectorAll('#rangeBar .range-btn').forEach(function (b) {
    b.addEventListener('click', function () {
      lidarRange = +b.getAttribute('data-range');
      placeThumb(); // 预设档位：推杆同步滑到对应位置
      markRangeButtons();
    });
  });

  // ---- 线性变焦推杆：纵拖连续缩放，松手回中归位 3m ----
  var zoomRail = $('zoomRail');
  var zoomThumb = $('zoomThumb');
  var dragging = false;

  function thumbCenter() {
    return (zoomRail.clientHeight - zoomThumb.offsetHeight) / 2;
  }
  function centerThumb() {
    zoomThumb.classList.remove('free');
    zoomThumb.classList.add('back');
    zoomThumb.style.top = thumbCenter() + 'px';
  }
  function thumbT() {
    // 推杆位移比：+1 顶（0.5m）~-1 底（6m），0 中线（3m），绝对映射
    var half = thumbCenter() || 1;
    var off = thumbCenter() - parseFloat(zoomThumb.style.top || thumbCenter());
    var t = off / half;
    return Math.max(-1, Math.min(1, t));
  }
  // 量程→推杆位置（thumbT 的逆映射）：预设档位点亮时推杆同步到位
  function placeThumb() {
    var t;
    if (lidarRange >= Z_DEF) t = -(lidarRange - Z_DEF) / (Z_MAX - Z_DEF);
    else t = (Z_DEF - lidarRange) / (Z_DEF - Z_MIN);
    t = Math.max(-1, Math.min(1, t));
    zoomThumb.classList.remove('free');
    zoomThumb.classList.add('back');
    zoomThumb.style.top = (thumbCenter() - t * (thumbCenter() || 1)) + 'px';
  }
  function applyZoom() {
    var t = thumbT();
    lidarRange = t >= 0 ? Z_DEF - t * (Z_DEF - Z_MIN) : Z_DEF - t * (Z_MAX - Z_DEF);
    markRangeButtons(); // 连续值无精确匹配时全部熄灭，归位 3m 自动点亮
  }
  function thumbMove(clientY) {
    var rail = zoomRail.getBoundingClientRect();
    var y = clientY - rail.top - zoomThumb.offsetHeight / 2;
    y = Math.max(0, Math.min(rail.height - zoomThumb.offsetHeight, y));
    zoomThumb.style.top = y + 'px';
    applyZoom();
  }
  zoomThumb.addEventListener('pointerdown', function (e) {
    e.preventDefault();
    dragging = true;
    zoomThumb.classList.remove('back');
    zoomThumb.classList.add('free');
    zoomThumb.setPointerCapture(e.pointerId);
  });
  zoomThumb.addEventListener('pointermove', function (e) {
    if (dragging) thumbMove(e.clientY);
  });
  function zoomEnd() {
    if (!dragging) return;
    dragging = false;
    // 松手保持当前量程：推杆停在释放位置，不再自动归位
    markRangeButtons();
  }
  zoomThumb.addEventListener('pointerup', zoomEnd);
  zoomThumb.addEventListener('pointercancel', zoomEnd);
  centerThumb();
  initLidar();
})();
