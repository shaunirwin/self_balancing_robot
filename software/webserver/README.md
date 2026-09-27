# Laptop dashboard

The current dashboard uses `index.html`, `main.js`, and Vite. Earlier control
interface experiments are kept in [`archive/`](archive/README.md).

Run from `software/webserver` with Node.js 20.19+ or 22.12+:

```sh
npm install
ROBOT_URL=http://192.168.178.55 npm run dev
```

Open <http://localhost:5173>. Set `ROBOT_URL` to the robot's HTTP origin on your network (no path or trailing slash). The default is `http://192.168.178.55`. Restart Vite after changing it. The server listens only on loopback. Browser API calls use `/api/*`; Vite forwards them to the robot with `/api` removed. The existing ESP32 endpoints are unchanged.

Open a local `SBRPB1` `.sbrpb` or legacy `SBRLOG1` `.bin` file in the recording viewer, or use Start, Stop, then Download & open. The new download uses the shared protobuf schema in `../../proto/recording.proto`. Click a chart or drag the timeline to inspect the corresponding pitch in the 3D view. Drag across a plot to zoom all charts to the same time range. Scroll over a plot to zoom around the pointer; Shift + scroll, middle-button drag, or the toolbar pans. Reset view restores the full recording. The view shows only pitch. With no recording selected, it follows live status polling. Controller values are submitted through `/set-value` as form fields; the dashboard rereads `/status` to confirm queued changes. Automatic mode may remain inactive if the robot's arm conditions are not met.

Expand **Recording settings** in the viewer for the snapshot captured at Start,
its firmware revision, IMU calibration validity, and the live tuning change
flag. Older recordings show "Metadata unavailable". Units and flag semantics
are documented in [`proto/README.md`](../../proto/README.md).

```sh
npm run build
```

The build checks and bundles the dashboard. For connected controls and recording, use `npm run dev` so the API proxy is active; opening `dist/index.html` directly does not provide a robot connection.
