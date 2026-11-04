/* fwq/config.js —— 控制台默认连接参数
 *
 * 换服务器时只改这一个文件，然后重新部署静态站点即可。
 * 页面里的输入框仍可临时覆盖，并会记在 localStorage（键名带 _v2，
 * 避免浏览器继续用旧服务器 IP）。
 *
 * 固件侧的 broker 可在运行时用 GET /api/mqtt?uri=mqtt://IP:端口&save=1
 * 修改，不需要重新烧录。
 */
window.FWQ_CONFIG = {
  brokerHost: '',        // 例：'203.0.113.10' 或 'mqtt.example.com'
  brokerPort: '8083',    // EMQX WebSocket 监听端口
  udpPort: 8888          // 雷达 UDP 广播端口（fwq/server.js 使用）
};
