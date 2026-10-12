import { defineConfig } from 'vite';
import vue from '@vitejs/plugin-vue';
import { viteSingleFile } from 'vite-plugin-singlefile';

// 单文件构建：JS/CSS 全部内联进 dist/index.html，
// 直接烧入 ESP32（替换 ld14_web.cc 的 kPage）或独立部署均可。

// 机器人目标地址（本地 dev 代理用）。支持环境变量覆盖：
//   ROBOT_IP=192.168.137.148 npm run dev
const ROBOT_TARGET = `http://${process.env.ROBOT_IP ?? '192.168.137.148'}`;

export default defineConfig({
  plugins: [vue(), viteSingleFile()],
  build: {
    outDir: 'dist',
    assetsInlineLimit: 100 * 1024 * 1024,
    chunkSizeWarningLimit: 1024,
    cssCodeSplit: false,
    minify: 'esbuild',
  },
  server: {
    // 本地联调：浏览器连同源 ws://localhost:5173/ws/lidar，
    // 由 Vite 底层桥接到机器人，抹平跨域与 Secure-Context 限制。
    // 机器人换 IP 只改 ROBOT_TARGET 一处（需重启 npm run dev）。
    proxy: {
      '/ws/lidar': {
        target: ROBOT_TARGET,
        ws: true,
        changeOrigin: true,
        timeout: 10000,
      },
      // 数据流 A 状态接口：同源 /api/lidar_status → 机器人 :80
      '/api/lidar_status': {
        target: ROBOT_TARGET,
        changeOrigin: true,
        timeout: 5000,
      },
      // 数据流 A 点云降级通道：同源 /api/lidar_points → 机器人 :80（桥接断开时 1Hz）
      '/api/lidar_points': {
        target: ROBOT_TARGET,
        changeOrigin: true,
        timeout: 5000,
      },
      // 电机测试台：同源 /api/motor/* → 机器人 :80
      '/api/motor': {
        target: ROBOT_TARGET,
        changeOrigin: true,
        timeout: 5000,
      },
    },
  },
});
