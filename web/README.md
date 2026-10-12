# 小智机器人中控台（Vue 3 + TS + Vite）

## 开发联调

```bash
cd web
npm install
npm run dev      # 浏览器开 http://localhost:5173，WS 代理到 192.168.137.199（见 vite.config.ts）
```

## 构建单文件并烧入 ESP32

```bash
cd web
npm run build    # 产物 web/dist/index.html（JS/CSS 全内联，约 150~250KB）
python embed.py  # 把 dist/index.html 转成 ld14_web.cc 的 kPage 常量并自动替换
```

`embed.py` 会备份原 `kPage` 为 `kPageLegacy`（保留旧简易页兜底），
新页面仍挂在 `GET /lidar`，WS 接口 `/ws/lidar` 不变：
下行沿用 `{ok,rpm,front,left,right,gmin,gangle,escape,pts[120]}`，
上行新增 `{cmd:"drive",v:-100..100,w:-100..100}`（20Hz）与 `{cmd:"stop"}`。

注意：单文件约 200KB 进 Flash（`.rodata`），app 分区 0x3F0000 需确认余量；
若超标，改走独立部署（平板直接 `npm run dev` 或 nginx 托管 dist）。
