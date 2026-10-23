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

  // ---------- 摇杆：x=转向, y=前进 ----------
  var joyEl = null, joyKnob = null, joyOn = false, joyX = 0, joyY = 0;
  function joyAction(l, r) {
    if (!l && !r) return 'stop';
    if (l === r) return l > 0 ? 'forward' : 'backward';
    if (l === -r) return l < 0 ? 'left' : 'right';
    return l > r ? 'right' : 'left';
  }
  function joyPush() {
    var l = Math.round(joyY * cruise + joyX * cruise);
    var r = Math.round(joyY * cruise - joyX * cruise);
    l = Math.max(-100, Math.min(100, l)); r = Math.max(-100, Math.min(100, r));
    intent = { action: joyAction(l, r), speed_l: l, speed_r: r };
    pub(intent);
    var jl = document.getElementById('joyL'), jr = document.getElementById('joyR');
    if (jl) jl.textContent = l; if (jr) jr.textContent = r;
  }
  function joySet(dx, dy) {
    joyX = Math.max(-1, Math.min(1, dx)); joyY = Math.max(-1, Math.min(1, dy));
    if (joyKnob) joyKnob.style.transform = 'translate(' + (joyX * 44) + 'px,' + (-joyY * 44) + 'px)';
    joyPush();
  }
  function joyReset() {
    joyX = 0; joyY = 0; joyOn = false;
    if (joyKnob) joyKnob.style.transform = '';
    halt(false);
    var jl = document.getElementById('joyL'), jr = document.getElementById('joyR');
    if (jl) jl.textContent = '0'; if (jr) jr.textContent = '0';
  }
  function joyInit() {
    joyEl = document.getElementById('joy'); joyKnob = document.getElementById('joyKnob');
    if (!joyEl || !joyKnob) return;
    function pos(e) {
      var r = joyEl.getBoundingClientRect();
      var cx = r.left + r.width / 2, cy = r.top + r.height / 2;
      var p = e.touches && e.touches[0] ? e.touches[0] : e;
      return [(p.clientX - cx) / (r.width / 2), (cy - p.clientY) / (r.height / 2)];
    }
    joyEl.addEventListener('pointerdown', function (e) { joyOn = true; joyEl.setPointerCapture(e.pointerId); var d = pos(e); joySet(d[0], d[1]); });
    joyEl.addEventListener('pointermove', function (e) { if (!joyOn) return; var d = pos(e); var m = Math.hypot(d[0], d[1]); if (m > 1) { d[0] /= m; d[1] /= m; } joySet(d[0], d[1]); });
    ['pointerup', 'pointercancel', 'pointerleave'].forEach(function (t) { joyEl.addEventListener(t, joyReset); });
  }
  joyInit();
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
    drive(act);
  });
  document.addEventListener('keyup', function (e) {
    var act = keyAct[e.code];
    if (!act || keyDown !== act) return;
    keyDown = null;
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
  var L_BG = '#f4f5f7';
  var L_FADE = 'rgba(244,245,247,0.3)';
  var L_GRID = '#c9d1dd';
  var L_TEXT = '#7c8798';
  var L_SELF = '#3e6ae1';

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
        ctx.fillStyle = d < 400 ? '#ff4444' : d < 1200 ? '#f5a623' : '#3e6ae1';
        var s = d < 400 ? 2 : 1;
        ctx.fillRect(x - s / 2, y - s / 2, s, s);
      }
      if (Date.now() - lidarAt > 1500) {
        $('lstatText').textContent = '点云超时（UDP :8888 是否到达本机？）';
      }
    }
    requestAnimationFrame(function () { drawLidar(cv); });
  }

  // ---------- CENTER 3D AUTOPILOT (Tesla-style) ----------
  var drive3dObs = [];
  var drive3dScene = null, drive3dCam = null, drive3dR = null, drive3dCar = null, drive3dWheels = [], drive3dPads = [], drive3dDashes = [], drive3dSolid = [];
  var drive3dAction = '待机';
  function initDrive3d() {
    if (typeof THREE === 'undefined') { var el = document.getElementById('driveStat'); if (el) el.textContent = 'Three.js 加载失败（检查网络）'; return; }
    var cv = document.getElementById('drive3d');
    drive3dR = new THREE.WebGLRenderer({ canvas: cv, antialias: true });
    drive3dScene = new THREE.Scene();
    drive3dScene.background = new THREE.Color(0x0b1220);
    drive3dScene.fog = new THREE.Fog(0x0b1220, 8, 26);
    drive3dCam = new THREE.PerspectiveCamera(58, 1, 0.1, 100);
    drive3dCam.position.set(0, 3.6, -5.2);
    drive3dCam.lookAt(0, 0.4, 3);
    drive3dScene.add(new THREE.AmbientLight(0xffffff, 1.1));
    var sun = new THREE.DirectionalLight(0xffffff, 1.1); sun.position.set(4, 8, -2); drive3dScene.add(sun);
    // 路面：深色可行驶带 + 两侧车道线 + 中心虚线
    var roadGeo = new THREE.PlaneGeometry(4.4, 60, 8, 120);
    var road = new THREE.Mesh(roadGeo, new THREE.MeshStandardMaterial({ color: 0x9AA1AB, roughness: 0.95 }));
    road.rotation.x = -Math.PI / 2; road.position.z = 18; drive3dScene.add(road);
    var roadBaseX = [], roadBaseY = [];
    (function(){ var pa = roadGeo.attributes.position; for (var vi = 0; vi < pa.count; vi++) { roadBaseX.push(pa.getX(vi)); roadBaseY.push(pa.getY(vi)); } })();
    var laneMat = new THREE.MeshBasicMaterial({ color: 0xffffff });
    drive3dDashes = []; drive3dSolid = [];
    // 两侧实线：连续面片（68 段细分），逐顶点弯曲成整根弧线，无接缝
    [-2.2, 2.2].forEach(function (x) {
      var g = new THREE.PlaneGeometry(0.07, 34, 1, 68);
      var m = new THREE.Mesh(g, laneMat);
      m.rotation.x = -Math.PI / 2; m.position.set(x, 0.02, 8);
      var pa = g.attributes.position;
      m.userData.x0 = x; m.userData.baseX = []; m.userData.baseY = [];
      for (var vi = 0; vi < pa.count; vi++) { m.userData.baseX.push(pa.getX(vi)); m.userData.baseY.push(pa.getY(vi)); }
      drive3dSolid.push(m); drive3dScene.add(m);
    });
    // 中间虚线：分段
    [-0.75, 0.75].forEach(function (x) {
      for (var di = 0; di < 10; di++) {
        var dg = new THREE.PlaneGeometry(0.07, 1.1);
        var dm = new THREE.Mesh(dg, laneMat);
        dm.rotation.x = -Math.PI / 2; dm.position.set(x, 0.02, -5 + di * 2.6);
        dm.userData.x0 = x;
        drive3dDashes.push(dm); drive3dScene.add(dm);
      }
    });
    // 两侧地面
    var side = new THREE.MeshStandardMaterial({ color: 0xdfe5ec, roughness: 1 });
    [-5, 5].forEach(function (x) { var p = new THREE.Mesh(new THREE.PlaneGeometry(14, 60), side); p.rotation.x = -Math.PI/2; p.position.set(x, -0.03, 18); drive3dScene.add(p); });
    // 本车：简洁履带车体 + 顶灯
    drive3dScene.background = new THREE.Color(0xdfe5ec);
    drive3dScene.fog = new THREE.Fog(0xdfe5ec, 8, 22);
    drive3dCar = new THREE.Group();
    var white = new THREE.MeshStandardMaterial({ color: 0xf2f4f6, roughness: 0.5, metalness: 0.15 });
    var darkM = new THREE.MeshStandardMaterial({ color: 0x2b2f36, roughness: 0.6, metalness: 0.3 });
    var trackM = new THREE.MeshStandardMaterial({ color: 0x3a3f47, roughness: 0.7, metalness: 0.4 });
    var padM = new THREE.MeshStandardMaterial({ color: 0x9aa3b2, roughness: 0.5, metalness: 0.5 });
    // 底盘：白色车体
    var chassis = new THREE.Mesh(new THREE.BoxGeometry(1.15, 0.5, 1.5), white);
    chassis.position.y = 0.55; drive3dCar.add(chassis);
    // 顶部盖板：白色，带圆形雷达开孔示意（深色圆环）
    var deck = new THREE.Mesh(new THREE.BoxGeometry(1.15, 0.1, 1.5), white);
    deck.position.y = 0.85; drive3dCar.add(deck);
    var hole = new THREE.Mesh(new THREE.CylinderGeometry(0.3, 0.3, 0.12, 24), darkM);
    hole.position.set(0, 0.86, -0.45); drive3dCar.add(hole);
    var hole2 = new THREE.Mesh(new THREE.CylinderGeometry(0.09, 0.09, 0.12, 16), darkM);
    hole2.position.set(-0.25, 0.86, 0.55); drive3dCar.add(hole2);
    // 前后白色电机舱圆筒
    [-0.62, 0.62].forEach(function (z) {
      var drum = new THREE.Mesh(new THREE.CylinderGeometry(0.17, 0.17, 0.7, 18), white);
      drum.rotation.z = Math.PI / 2; drum.position.set(0, 0.28, z); drive3dCar.add(drum);
    });
    // 两侧履带：深色带 + 浅色履带板
    [-0.75, 0.75].forEach(function (x) {
      var t = new THREE.Mesh(new THREE.BoxGeometry(0.3, 0.42, 1.8), trackM);
      t.position.set(x, 0.28, 0); drive3dCar.add(t);
      for (var w = 0; w < 9; w++) {
        var pad = new THREE.Mesh(new THREE.BoxGeometry(0.32, 0.06, 0.14), padM);
        pad.position.set(x, 0.5, -0.8 + w * 0.2); pad.userData.side = x < 0 ? -1 : 1; drive3dCar.add(pad); pad.userData.track=1; drive3dPads.push(pad);
        var pad2 = pad.clone(); pad2.position.y = 0.06; pad2.userData.side = x < 0 ? -1 : 1; pad2.userData.track=1; drive3dCar.add(pad2); drive3dPads.push(pad2);
      }
      // 负重轮
      for (var k = 0; k < 4; k++) {
        var wh = new THREE.Mesh(new THREE.CylinderGeometry(0.14, 0.14, 0.32, 16), darkM);
        wh.rotation.z = Math.PI / 2; wh.position.set(x, 0.2, -0.55 + k * 0.37); wh.userData.side = x < 0 ? -1 : 1; drive3dCar.add(wh); drive3dWheels.push(wh);
      }
    });
    var ring = new THREE.Mesh(new THREE.RingGeometry(1.15, 1.25, 48), new THREE.MeshBasicMaterial({ color: 0x9aa3b2, transparent: true, opacity: 0.6, side: THREE.DoubleSide }));
    ring.rotation.x = -Math.PI / 2; ring.position.y = 0.02; drive3dCar.add(ring);
    drive3dScene.add(drive3dCar);
    // 蓝色预测轨迹带（特斯拉式弧形）
    var PATH_N = 40, PATH_LEN = 9;
    var pathGeo = new THREE.PlaneGeometry(0.5, PATH_LEN / PATH_N, 1, PATH_N);
    var pathMat = new THREE.MeshBasicMaterial({ color: 0x2f6bff, transparent: true, opacity: 0.55, side: THREE.DoubleSide, depthWrite: false });
    var pathRibbon = new THREE.Mesh(pathGeo, pathMat);
    pathRibbon.rotation.x = -Math.PI / 2;
    pathRibbon.position.y = 0.02;
    pathRibbon.renderOrder = 1;
    drive3dScene.add(pathRibbon);
    function updatePathRibbon() {
      var sL = (intent.speed_l || 0) / 100, sR = (intent.speed_r || 0) / 100;
      var on = intent.action !== 'stop' && intent.action !== 'brake' && (sL || sR);
      var mean = (sL + sR) / 2;
      var yaw = drive3dCar.rotation.y / 0.6; // 约 -1..1
      var bend = yaw * 4.5; // 远端横向偏移
      var pp = pathGeo.attributes.position;
      for (var j = 0; j < pp.count; j++) {
        var row = Math.floor(j / 2) / PATH_N;
        var bx = bend * row * row;
        var ww = 0.32 * (1 - row * 0.55);
        pp.setX(j, bx + (j % 2 === 0 ? -ww / 2 : ww / 2));
        pp.setY(j, -1.2 - row * PATH_LEN + PATH_LEN / 2);
      }
      pp.needsUpdate = true;
      pathGeo.computeVertexNormals();
      pathRibbon.visible = on && (Math.abs(yaw) > 0.03 || Math.abs(mean) > 0.03);
    }
    // 障碍物池：雷达点云聚类后显示为红色柱体，最多 24 个
    for (var i = 0; i < 24; i++) {
      var g = new THREE.Group();
      var b = new THREE.Mesh(new THREE.BoxGeometry(0.9, 0.35, 1.6), new THREE.MeshStandardMaterial({ color: 0xe8ebf0, roughness: 0.5 }));
      b.position.y = 0.35; g.add(b);
      var c2 = new THREE.Mesh(new THREE.BoxGeometry(0.7, 0.25, 0.8), new THREE.MeshStandardMaterial({ color: 0x3a3f47, roughness: 0.3 }));
      c2.position.set(0, 0.6, -0.1); g.add(c2);
      g.visible = false; drive3dScene.add(g); drive3dObs.push(g);
    }
    function resize() {
      var w = cv.parentElement.clientWidth || 300, h = cv.parentElement.clientHeight || 300;
      drive3dR.setSize(w, h, false); drive3dCam.aspect = w / h; drive3dCam.updateProjectionMatrix();
    }
    window.addEventListener('resize', resize); resize();
    (function loop() {
      requestAnimationFrame(loop);
      var spdL = (intent.speed_l || 0) / 100, spdR = (intent.speed_r || 0) / 100;
      var diff = spdR - spdL;
      var tgt = Math.max(-1, Math.min(1, diff / 2)) * 0.6;
      drive3dCar.rotation.y += (tgt - drive3dCar.rotation.y) * 0.12;
      var active = intent.action !== 'stop' && intent.action !== 'brake' && (spdL || spdR);
      function sideSpd(s) { return s < 0 ? spdL : spdR; }
      drive3dWheels.forEach(function (w) { var s = sideSpd(w.userData.side || 1); if (active && s) w.rotation.x += s * 0.07; });
      drive3dPads.forEach(function (p) { var s = sideSpd(p.userData.side || 1); if (active && s) { p.position.z += s * 0.01; if (p.position.z > 0.9) p.position.z = -0.9; if (p.position.z < -0.9) p.position.z = 0.9; } });
      drive3dCar.position.z = active ? Math.sin(Date.now()/300)*0.03 : 0;
      var avg = (spdL + spdR) / 2;
      // 白线/虚线与蓝带同曲率弯曲
      var yawN = drive3dCar.rotation.y / 0.6;
      function laneBend(z) {
        var t = Math.max(0, Math.min(1, (z - 1.2) / 16));
        return { x: yawN * 4.5 * t * t, a: yawN * 0.56 * t };
      }
      // 主路面逐顶点弯曲（与白线同曲率）
      (function(){
        var pos = roadGeo.attributes.position;
        for (var ri = 0; ri < pos.count; ri++) {
          var wz = 18 - roadBaseY[ri];
          var tt = Math.max(0, Math.min(1, (wz - 1.2) / 16));
          pos.setX(ri, roadBaseX[ri] + yawN * 4.5 * tt * tt);
        }
        pos.needsUpdate = true;
      })();
      // 实线逐顶点弯曲（整根弧线）
      drive3dSolid.forEach(function (m) {
        var pos = m.geometry.attributes.position;
        for (var vi = 0; vi < pos.count; vi++) {
          var wz = 8 - m.userData.baseY[vi];
          pos.setX(vi, m.userData.baseX[vi] + laneBend(wz).x);
        }
        pos.needsUpdate = true;
      });
      drive3dDashes.forEach(function (m) {
        if (active && avg) {
          m.position.z += -avg * 0.06;
          if (m.position.z > 8) m.position.z -= 26;
          if (m.position.z < -18) m.position.z += 26;
        }
        var b = laneBend(m.position.z);
        m.position.x = m.userData.x0 + b.x;
        m.rotation.z = b.a; // 与位移同向，顺弧线切线
      });
      // 蓝色预测轨迹带：弧形，随摇杆转向弯曲
      updatePathRibbon();
      refreshDrive3dObs();
      drive3dR.render(drive3dScene, drive3dCam);
      var sp = Math.max(Math.abs(intent.speed_l), Math.abs(intent.speed_r));
      var hs = document.getElementById('hudSpeed'); if (hs) hs.textContent = sp;
      var hd = document.getElementById('hudDir');
      if (hd) hd.textContent = intent.action === 'forward' ? '前进' : intent.action === 'backward' ? '后退' : intent.action === 'left' ? '左转' : intent.action === 'right' ? '右转' : intent.action === 'brake' ? '急停' : '待机';
    })();
  }
  function refreshDrive3dObs() {
    if (!drive3dScene) return;
    // 取雷达最近点做降采样聚类：每 15° 取最近点映射到车前扇区
    var buckets = {};
    for (var i = 0; i < lidarPts.length; i++) {
      var a = lidarPts[i][0], d = lidarPts[i][1];
      if (d > 4000 || d < 150) continue;
      var k = Math.floor(a / 15);
      if (!buckets[k] || d < buckets[k].d) buckets[k] = { a: a, d: d };
    }
    var keys = Object.keys(buckets).slice(0, 24);
    for (var j = 0; j < drive3dObs.length; j++) {
      var o = drive3dObs[j];
      if (j >= keys.length) { o.visible = false; continue; }
      var b = buckets[keys[j]];
      var rad = b.a * Math.PI / 180, rr = Math.min(b.d / 1000, 6);
      o.visible = true;
      o.position.set(Math.sin(rad) * rr, 0.45, Math.cos(rad) * rr);
      var s = b.d < 600 ? 1.3 : 1;
      o.scale.set(s, 1, s);
    }
  }
  try { initDrive3d(); } catch (e) { var d = document.getElementById('driveStat'); if (d) d.textContent = '3D 初始化失败: ' + (e && e.message); if (typeof console!=='undefined') console.error(e); }

  // RPM mini chart
  var rpmHistA=[], rpmHistB=[];
  setInterval(function(){
    try{
      var cv=document.getElementById('rpmChart'); if(!cv) return;
      var a=parseFloat((document.getElementById('rpmA')||{}).textContent)||0;
      var b=parseFloat((document.getElementById('rpmB')||{}).textContent)||0;
      rpmHistA.push(a); rpmHistB.push(b);
      if(rpmHistA.length>60) rpmHistA.shift(); if(rpmHistB.length>60) rpmHistB.shift();
      var c=cv.getContext('2d'); var W=cv.width=cv.clientWidth||300, H=cv.height=120;
      c.clearRect(0,0,W,H); c.strokeStyle='#e2e7ec';
      for(var g=1; g<4; g++){ c.beginPath(); c.moveTo(0,H*g/4); c.lineTo(W,H*g/4); c.stroke(); }
      function line(arr,col){ c.strokeStyle=col; c.lineWidth=2; c.beginPath();
        var mx=Math.max.apply(null,arr.concat([1]));
        arr.forEach(function(v,i){ var x=i/59*W, y=H-8-(v/mx)*(H-20); i?c.lineTo(x,y):c.moveTo(x,y); }); c.stroke(); }
      line(rpmHistA,'#0e9f6e'); line(rpmHistB,'#3e6ae1');
      var e1=document.getElementById('rpmA2'); if(e1) e1.textContent=document.getElementById('rpmA').textContent;
      var e2=document.getElementById('rpmB2'); if(e2) e2.textContent=document.getElementById('rpmB').textContent;
    }catch(e){}
  },500);
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
