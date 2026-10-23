(function () {
  'use strict';
  var $ = function (id) { return document.getElementById(id); };
  var client = null;
  var connected = false;
  var command = { action: 'stop', speed_l: 0, speed_r: 0 };
  var lastFeedback = 0;
  var action = null;
  var diameterMm = 68;
  var telemetry = { l: 0, r: 0, countL: 0, countR: 0 };
  var lastCounts = null;
  var reconnectTimer = null;

  function setStatus(el, text, kind) {
    el.textContent = text;
    el.className = 'status ' + (kind || 'offline');
  }
  function publish(payload) {
    if (!connected || !client) return false;
    client.publish('robot/cmd', JSON.stringify(payload), { qos: 0 });
    return true;
  }
  function stop(reason) {
    command = { action: 'stop', speed_l: 0, speed_r: 0 };
    publish(command);
    cancelDistance(reason || '已停止');
    setTrack(0, 0);
    setJoystick(0, 0);
  }
  function connect() {
    if (connected && client) { client.end(); return; }
    var url = 'ws://' + ($('brokerIp').value.trim() || '103.231.59.46') + ':' + ($('brokerPort').value.trim() || '8083') + '/mqtt';
    setStatus($('state'), '连接中', 'warn');
    client = mqtt.connect(url, { connectTimeout: 5000, reconnectPeriod: 3000 });
    client.on('connect', function () {
      connected = true;
      $('connectBtn').textContent = '断开';
      setStatus($('state'), '在线', 'online');
      client.subscribe('robot/telemetry', { qos: 0 });
    });
    client.on('message', function (topic, msg) {
      if (topic !== 'robot/telemetry') return;
      try {
        var data = JSON.parse(msg.toString());
        telemetry = { l: Number(data.rpm_l) || 0, r: Number(data.rpm_r) || 0, countL: Number(data.cnt_l) || 0, countR: Number(data.cnt_r) || 0 };
        $('rpmL').textContent = telemetry.l.toFixed(1); $('rpmR').textContent = telemetry.r.toFixed(1);
        $('countL').textContent = telemetry.countL; $('countR').textContent = telemetry.countR;
        lastFeedback = Date.now(); setStatus($('feedbackState'), '正常', 'online');
        updateDistanceProgress();
      } catch (e) { setStatus($('feedbackState'), '数据错误', 'warn'); }
    });
    client.on('close', function () { connected = false; $('connectBtn').textContent = '连接'; setStatus($('state'), '离线', 'offline'); if (action) cancelDistance('连接断开'); });
    client.on('error', function () { setStatus($('state'), '连接错误', 'warn'); });
  }
  function sendManual(left, right) {
    command = { action: 'manual', speed_l: Math.round(left), speed_r: Math.round(right) };
    publish(command);
  }
  function setTrack(side, value) {
    value = Math.max(-100, Math.min(100, Number(value) || 0));
    if (side === 'left') { $('leftSlider').value = value; $('leftValue').textContent = value + '%'; }
    else { $('rightSlider').value = value; $('rightValue').textContent = value + '%'; }
    sendManual(Number($('leftSlider').value), Number($('rightSlider').value));
  }
  function setJoystick(v, w) {
    v = Math.max(-100, Math.min(100, v)); w = Math.max(-100, Math.min(100, w));
    var left = Math.max(-100, Math.min(100, v - w)); var right = Math.max(-100, Math.min(100, v + w));
    $('joyV').textContent = Math.round(v); $('joyW').textContent = Math.round(w); $('joyL').textContent = Math.round(left); $('joyR').textContent = Math.round(right);
    var max = 75, x = w / 100 * max, y = -v / 100 * max;
    $('knob').style.transform = 'translate(' + x + 'px,' + y + 'px)';
    if (v === 0 && w === 0) sendManual(0, 0); else sendManual(left, right);
  }
  function countsPerRev() { return Math.max(1, Number($('countsPerRev').value) || 3344); }
  function refreshDistance() {
    var mm = Math.max(1, Number($('distance').value) || 1); var rev = mm / (Math.PI * diameterMm); var counts = Math.round(rev * countsPerRev());
    $('revolutions').textContent = rev.toFixed(3); $('targetCounts').textContent = counts;
  }
  function startDistance() {
    if (!connected) { $('distanceStatus').textContent = '请先连接 Broker'; return; }
    var mm = Math.max(1, Math.min(100000, Number($('distance').value) || 0)); var speed = Math.max(5, Math.min(100, Number($('distanceSpeed').value) || 0));
    if (!mm || !speed) return;
    var dir = Number($('distanceDir').value); var target = Math.round(mm / (Math.PI * diameterMm) * countsPerRev());
    lastCounts = { l: telemetry.countL, r: telemetry.countR };
    action = { target: Math.abs(target), dir: dir, speed: speed, started: Date.now(), maxMs: Math.max(5000, mm / 10 * 1000 + 5000) };
    $('distanceStatus').textContent = '运行中：两侧同步计数，接近目标会停止'; $('distanceProgress').style.width = '0%';
    command = { action: 'manual', speed_l: dir * speed, speed_r: dir * speed }; publish(command);
  }
  function cancelDistance(reason) { if (!action) return; action = null; $('distanceStatus').textContent = reason || '动作已取消'; $('distanceProgress').style.width = '0%'; }
  function updateDistanceProgress() {
    if (!action || !lastCounts) return;
    var dl = Math.abs(telemetry.countL - lastCounts.l), dr = Math.abs(telemetry.countR - lastCounts.r), done = Math.min(dl, dr), progress = Math.min(100, done / action.target * 100);
    $('distanceProgress').style.width = progress.toFixed(1) + '%';
    var wrong = (telemetry.countL - lastCounts.l) * action.dir < -20 || (telemetry.countR - lastCounts.r) * action.dir < -20;
    if (wrong) { stop('检测到编码器方向与动作方向不一致'); return; }
    if (done >= action.target) stop('距离动作完成');
  }
  function tick() {
    if (action) {
      if (Date.now() - action.started > action.maxMs) stop('距离动作超时');
      else if (Date.now() - lastFeedback > 1500) stop('遥测反馈超时');
      else publish(command);
    } else if (connected && command.action !== 'stop') publish(command);
  }
  function bindJoystick() {
    var joy = $('joystick'), active = false;
    function move(e) { if (!active) return; var r = joy.getBoundingClientRect(), x = (e.clientX - (r.left + r.width / 2)) / (r.width / 2), y = (e.clientY - (r.top + r.height / 2)) / (r.height / 2); var len = Math.hypot(x, y); if (len > 1) { x /= len; y /= len; } setJoystick(-y * 100, x * 100); }
    joy.addEventListener('pointerdown', function (e) { active = true; joy.setPointerCapture(e.pointerId); move(e); });
    joy.addEventListener('pointermove', move); joy.addEventListener('pointerup', function () { active = false; setJoystick(0, 0); }); joy.addEventListener('pointercancel', function () { active = false; setJoystick(0, 0); });
  }
  $('connectBtn').addEventListener('click', connect); $('stopBtn').addEventListener('click', function () { stop('急停'); });
  $('distanceStart').addEventListener('click', startDistance); $('distanceCancel').addEventListener('click', function () { stop('动作已取消'); });
  ['distance', 'countsPerRev'].forEach(function (id) { $(id).addEventListener('input', refreshDistance); });
  $('leftSlider').addEventListener('input', function () { setTrack('left', this.value); }); $('rightSlider').addEventListener('input', function () { setTrack('right', this.value); });
  document.querySelectorAll('[data-track]').forEach(function (b) { b.addEventListener('click', function () { setTrack(b.getAttribute('data-track'), b.getAttribute('data-value')); }); });
  $('centerTracks').addEventListener('click', function () { stop('双履带已停止'); });
  window.addEventListener('beforeunload', function () { publish({ action: 'stop', speed_l: 0, speed_r: 0 }); });
  setInterval(tick, 200); bindJoystick(); refreshDistance();
})();
