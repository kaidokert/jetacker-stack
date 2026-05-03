#!/usr/bin/env python3
"""
Host-side FastAPI orchestrator for the robot dashboard.

Replaces the in-container dashboard API. Runs on the host where it can
import stack.py, drive_utils, and reset_orchestrator directly — no docker
CLI needed inside a container.

Usage:
    python dashboard/api/host_api.py

Or:
    python -m uvicorn dashboard.api.host_api:app --host 0.0.0.0 --port 8080
"""

import asyncio
import logging
import os
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse, JSONResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel

# ── Path setup ────────────────────────────────────────────────────────────────

PROJECT_ROOT = Path(__file__).resolve().parent.parent.parent  # dashboard/api/
sys.path.insert(0, str(PROJECT_ROOT))
sys.path.insert(0, str(PROJECT_ROOT / "tools"))

import stack  # noqa: E402

# ── App ───────────────────────────────────────────────────────────────────────

logger = logging.getLogger("host_api")

app = FastAPI(title="Robot Dashboard API (host-side)", version="0.2.0")

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Two workers: one for mutations (teleport/reset), one for reads (status/params)
# Prevents a slow mutation from blocking status queries.
_stack_executor = ThreadPoolExecutor(max_workers=2, thread_name_prefix="stack")

# Default timeout for docker compose exec calls (seconds)
_EXEC_TIMEOUT = 15


async def _run_in_stack_thread(fn, *args):
    """Run a blocking stack function in the dedicated thread pool."""
    loop = asyncio.get_event_loop()
    return await loop.run_in_executor(_stack_executor, fn, *args)


# ── Stack helpers ─────────────────────────────────────────────────────────────


def _safe_soft_reset() -> dict:
    """Call stack.soft_reset() with SystemExit protection."""
    t0 = time.time()
    try:
        stack.soft_reset()
        dt = time.time() - t0
        return {"success": True, "message": f"Soft reset complete ({dt:.1f}s)"}
    except SystemExit as e:
        dt = time.time() - t0
        return {"success": False, "message": f"Soft reset failed (exit code {e.code}, {dt:.1f}s)"}
    except Exception as e:
        dt = time.time() - t0
        return {"success": False, "message": f"Soft reset error: {e} ({dt:.1f}s)"}


def _teleport_to_origin() -> dict:
    """Teleport robot to origin.

    1. Gazebo: pause → teleport → zero joints → unpause (docker exec into gazebo — gz CLI only there)
    2. ROS2 resets: cancel Nav2 goal, reset EKF pose, sleep 1s, reset AMCL (single docker exec into test-drive)
    """
    t0 = time.time()
    try:
        manifest = stack.load_manifest()
        detected = stack._detect_running_stack(manifest)
        if detected is None:
            return {"success": False, "message": "No running stack detected"}

        robot_name, stack_name, running_services = detected
        gazebo_service = stack.component_to_service(manifest, robot_name, "gazebo")

        stack._load_dotenv()
        world_file = os.environ.get("JETACKER_WORLD", "jetacker/jetacker.sdf")
        world_name = Path(world_file).stem
        gz_world = f"{world_name}_world"
        model_name = robot_name

        joint_reset_bin = "/workspace/gz_plugins/world_state_publisher/build/joint_reset"

        env = os.environ.copy()
        env["MSYS_NO_PATHCONV"] = "1"

        # Step 1: Gazebo commands (gz CLI only exists in gazebo container)
        gz_cmds = " && ".join([
            f'gz service -s /world/{gz_world}/control '
            f'--reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean '
            f'--timeout 5000 --req "pause: true"',

            f'gz service -s /world/{gz_world}/set_pose '
            f'--reqtype gz.msgs.Pose --reptype gz.msgs.Boolean '
            f"""--timeout 5000 --req 'name: "{model_name}" """
            f"""position {{ x: 0 y: 0 z: 0.05 }} orientation {{ w: 1.0 }}'""",

            f"{joint_reset_bin} {model_name}",

            f'gz service -s /world/{gz_world}/control '
            f'--reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean '
            f'--timeout 5000 --req "pause: false"',
        ])

        result = subprocess.run(
            ["docker", "compose", "exec", "-T", gazebo_service,
             "bash", "-c", f"source /opt/ros/jazzy/setup.bash && {gz_cmds}"],
            capture_output=True, text=True, env=env,
            cwd=str(PROJECT_ROOT), timeout=_EXEC_TIMEOUT,
        )
        if result.returncode != 0:
            dt = time.time() - t0
            detail = result.stderr[:300] if result.stderr else "(no output)"
            return {"success": False, "message": f"Teleport failed ({dt:.1f}s): {detail}"}

        # Step 2: ROS2 resets — single exec into test-drive (all containers share namespace)
        ros2_cmds = ["source /opt/ros/jazzy/setup.bash"]

        # Cancel any active Nav2 goal (best-effort)
        ros2_cmds.append(
            "ros2 service call /navigate_to_pose/_action/cancel_goal "
            "action_msgs/srv/CancelGoal '{}' || true"
        )

        # Reset EKF pose to origin
        ekf_service = stack.component_to_service(manifest, robot_name, "ekf_localization")
        if ekf_service in running_services:
            ros2_cmds.append(
                "ros2 service call /set_pose robot_localization/srv/SetPose "
                '"{pose: {header: {frame_id: odom}, pose: {pose: '
                '{position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}}"'
            )

        # AMCL reset — after 1s sleep for TF settle
        amcl_service = stack.component_to_service(manifest, robot_name, "amcl")
        if amcl_service in running_services:
            ros2_cmds.append("sleep 1")
            ros2_cmds.append(
                "ros2 topic pub --once /initialpose "
                "geometry_msgs/msg/PoseWithCovarianceStamped "
                '"{header: {frame_id: map}, pose: {pose: '
                '{position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}"'
            )

        subprocess.run(
            ["docker", "compose", "exec", "-T", "test-drive",
             "bash", "-c", " && ".join(ros2_cmds)],
            capture_output=True, text=True, env=env,
            cwd=str(PROJECT_ROOT), timeout=_EXEC_TIMEOUT,
        )

        dt = time.time() - t0
        return {"success": True, "message": f"Teleported to origin ({dt:.1f}s)"}

    except subprocess.TimeoutExpired:
        dt = time.time() - t0
        return {"success": False, "message": f"Teleport timed out ({dt:.1f}s)"}
    except Exception as e:
        dt = time.time() - t0
        return {"success": False, "message": f"Teleport error: {e} ({dt:.1f}s)"}


def _get_stack_status() -> dict:
    """Get running stack info."""
    try:
        manifest = stack.load_manifest()
        running = stack.get_running_services()
        detected = stack._detect_running_stack(manifest)
        return {
            "running_services": running,
            "detected_stack": f"{detected[0]}:{detected[1]}" if detected else None,
            "robot": detected[0] if detected else None,
        }
    except Exception as e:
        return {"running_services": [], "detected_stack": None, "error": str(e)}


# ── Endpoints ─────────────────────────────────────────────────────────────────


@app.get("/api/health")
def health() -> dict:
    return {"status": "ok"}


@app.get("/api/config")
def config() -> dict:
    return {
        "rosbridge_url": os.environ.get("ROSBRIDGE_URL", "ws://localhost:9090"),
    }


@app.get("/api/stack/status")
async def api_stack_status():
    return await _run_in_stack_thread(_get_stack_status)


@app.post("/api/stack/soft-reset")
async def api_soft_reset():
    result = await _run_in_stack_thread(_safe_soft_reset)
    code = 200 if result["success"] else 500
    return JSONResponse(content=result, status_code=code)



# NOTE: Planner params and clutch are now handled directly via rosbridge
# from the frontend (no docker exec needed). Only teleport/retry/soft-reset
# need docker exec (for gz CLI commands).


@app.post("/api/mppi/teleport")
async def api_teleport():
    result = await _run_in_stack_thread(_teleport_to_origin)
    code = 200 if result["success"] else 500
    return JSONResponse(content=result, status_code=code)


class RetryRequest(BaseModel):
    waypoints: str = "nav2_matrix_3_forward_left_90"


@app.post("/api/mppi/retry")
async def api_retry(req: RetryRequest):
    # Step 1: soft reset
    reset_result = await _run_in_stack_thread(_safe_soft_reset)
    if not reset_result["success"]:
        return JSONResponse(
            content={"status": "error", "step": "soft-reset", "detail": reset_result["message"]},
            status_code=500,
        )

    # Step 2: send Nav2 goal via test-drive container
    goal_cmd = (
        "source /opt/ros/jazzy/setup.bash && "
        f"ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "
        "\"$(python3 -c \""
        "import yaml, json, sys; "
        f"wps=yaml.safe_load(open('/workspace/driving_instructions/{req.waypoints}.yaml')); "
        "wp=wps['waypoints'][-1]; "
        "pose={'header':{'frame_id':'map'},'pose':{'position':{'x':wp['x'],'y':wp['y'],'z':0.0},"
        "'orientation':{'x':0,'y':0,'z':wp.get('qz',0),'w':wp.get('qw',1)}}}; "
        "print(json.dumps({'pose':pose}))"
        "\")\""
    )

    env = os.environ.copy()
    env["MSYS_NO_PATHCONV"] = "1"

    try:
        proc = await asyncio.create_subprocess_exec(
            "docker", "compose", "exec", "-T", "test-drive",
            "bash", "-c", goal_cmd,
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.PIPE,
            cwd=str(PROJECT_ROOT),
            env=env,
        )
        stdout, stderr = await asyncio.wait_for(proc.communicate(), timeout=30)
        if proc.returncode != 0:
            return JSONResponse(
                content={
                    "status": "error",
                    "step": "send-goal",
                    "detail": stderr.decode(errors="replace")[-500:],
                },
                status_code=500,
            )
    except asyncio.TimeoutError:
        return JSONResponse(
            content={"status": "error", "step": "send-goal", "detail": "Timed out after 30s"},
            status_code=500,
        )

    return {"status": "ok", "waypoints": req.waypoints}


@app.get("/api/waypoints")
def list_waypoints():
    wp_dir = PROJECT_ROOT / "driving_instructions"
    if not wp_dir.exists():
        return {"waypoints": []}
    files = sorted(p.stem for p in wp_dir.glob("nav2_*.yaml"))
    return {"waypoints": files}


# ── Static files ──────────────────────────────────────────────────────────────

STATIC_DIR = PROJECT_ROOT / "dashboard" / "web" / "dist"

if STATIC_DIR.exists():
    # Mount assets first (so /assets/* resolves)
    assets_dir = STATIC_DIR / "assets"
    if assets_dir.exists():
        app.mount("/assets", StaticFiles(directory=assets_dir), name="assets")

    @app.get("/")
    def index():
        return FileResponse(STATIC_DIR / "index.html")


else:
    @app.get("/")
    def index_fallback():
        return {
            "message": "Dashboard not built yet. Run: cd dashboard/web && npm run build",
        }


# ── Entry point ───────────────────────────────────────────────────────────────

if __name__ == "__main__":
    import uvicorn

    os.chdir(str(PROJECT_ROOT))
    print(f"Project root: {PROJECT_ROOT}")
    print(f"Static dir:   {STATIC_DIR} ({'exists' if STATIC_DIR.exists() else 'MISSING'})")
    port = int(os.environ.get("DASHBOARD_PORT", "8080"))
    print(f"Starting on http://0.0.0.0:{port}")
    print(f"If port {port} is busy, set DASHBOARD_PORT=8081")
    uvicorn.run(app, host="0.0.0.0", port=port)
