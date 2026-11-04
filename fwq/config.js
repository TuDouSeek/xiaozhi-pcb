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
  brokerHost: '103.236.92.193',  // NAT 外部 IP
  brokerPort: '38488',           // NAT 外部 WebSocket 端口（内部 8083）
  udpPort: 8888                  // 雷达 UDP 广播端口（fwq/server.js 使用）
};
