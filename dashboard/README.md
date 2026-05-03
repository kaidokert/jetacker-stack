# Dashboard (Phase 1)

Phase 1 provides read-only ROS2 observability for:

- `/bt_navigator/behavior_tree_log`
- `/diagnostics` (WARN/ERROR feed)

## Local dev

1. Start your sim/nav2 stack and rosbridge.
2. Run API:

```bash
cd dashboard/api
python -m venv .venv
. .venv/bin/activate
pip install -r requirements.txt
uvicorn main:app --reload --port 8080
```

3. Run web:

```bash
cd dashboard/web
npm install
npm run dev
```

Web URL: `http://localhost:5173`

## Environment

- `VITE_ROSBRIDGE_URL` (optional): override rosbridge WS URL in frontend.
- `ROSBRIDGE_URL` (optional): exposed by `/api/config` for future backend-driven config.
