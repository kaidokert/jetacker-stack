import asyncio
import logging
import os
import subprocess
from pathlib import Path

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel

logger = logging.getLogger("dashboard")

app = FastAPI(title="Robot Dashboard API", version="0.1.0")

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

STATIC_DIR = Path("/app/static")


@app.get("/api/health")
def health() -> dict[str, str]:
    return {"status": "ok"}


@app.get("/api/config")
def config() -> dict[str, str]:
    return {
        "rosbridge_url": os.environ.get("ROSBRIDGE_URL", "ws://localhost:9090"),
    }


## --- MPPI Retry Endpoint ---

# Project root: either WORKSPACE_DIR env var or /workspace (inside container)
WORKSPACE_DIR = Path(os.environ.get("WORKSPACE_DIR", "/workspace"))


GZ_WORLD = os.environ.get("GZ_WORLD", "jetacker_world")
GZ_MODEL = os.environ.get("GZ_MODEL", "jetacker")
GZ_SERVICE = os.environ.get("GZ_SERVICE", "jetacker-gazebo")


async def _gz_exec(gz_cmd: str, timeout: float = 15) -> dict:
    """Run a gz command inside the Gazebo container."""
    env = os.environ.copy()
    env["MSYS_NO_PATHCONV"] = "1"
    try:
        proc = await asyncio.create_subprocess_exec(
            "docker", "compose", "exec", "-T", GZ_SERVICE,
            "bash", "-c", f"source /opt/ros/jazzy/setup.bash && {gz_cmd}",
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.PIPE,
            cwd=str(WORKSPACE_DIR),
            env=env,
        )
        stdout, stderr = await asyncio.wait_for(proc.communicate(), timeout=timeout)
        if proc.returncode != 0:
            msg = stderr.decode(errors="replace")[-500:]
            return {"success": False, "message": msg, "status": "error", "detail": msg}
        return {"success": True, "message": "OK", "status": "ok"}
    except asyncio.TimeoutError:
        msg = f"Timed out after {timeout}s"
        return {"success": False, "message": msg, "status": "error", "detail": msg}
    except FileNotFoundError:
        msg = "docker not found"
        return {"success": False, "message": msg, "status": "error", "detail": msg}


@app.post("/api/mppi/teleport")
async def mppi_teleport() -> dict:
    """Teleport the robot back to origin. No pause/unpause, no joint reset."""
    return await _gz_exec(
        f'gz service -s /world/{GZ_WORLD}/set_pose '
        f'--reqtype gz.msgs.Pose --reptype gz.msgs.Boolean '
        f'--timeout 5000 '
        f"""--req 'name: "{GZ_MODEL}" position {{ x: 0 y: 0 z: 0.05 }} orientation {{ w: 1.0 }}'"""
    )


class RetryRequest(BaseModel):
    waypoints: str = "nav2_matrix_3_forward_left_90"


@app.post("/api/mppi/retry")
async def mppi_retry(req: RetryRequest) -> dict:
    """Soft-reset the robot and send a Nav2 goal."""
    # Step 1: soft-reset via stack.py
    stack_py = WORKSPACE_DIR / "stack.py"
    try:
        proc = await asyncio.create_subprocess_exec(
            "python",
            str(stack_py),
            "soft-reset",
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.PIPE,
            cwd=str(WORKSPACE_DIR),
        )
        stdout, stderr = await asyncio.wait_for(proc.communicate(), timeout=30)
        if proc.returncode != 0:
            return {
                "status": "error",
                "step": "soft-reset",
                "detail": stderr.decode(errors="replace")[-500:],
            }
    except asyncio.TimeoutError:
        return {"status": "error", "step": "soft-reset", "detail": "Timed out after 30s"}
    except FileNotFoundError:
        return {"status": "error", "step": "soft-reset", "detail": f"stack.py not found at {stack_py}"}

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
    try:
        proc = await asyncio.create_subprocess_exec(
            "docker", "compose", "exec", "-T", "test-drive",
            "bash", "-c", goal_cmd,
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.PIPE,
            cwd=str(WORKSPACE_DIR),
        )
        stdout, stderr = await asyncio.wait_for(proc.communicate(), timeout=30)
        if proc.returncode != 0:
            return {
                "status": "error",
                "step": "send-goal",
                "detail": stderr.decode(errors="replace")[-500:],
            }
    except asyncio.TimeoutError:
        return {"status": "error", "step": "send-goal", "detail": "Timed out after 30s"}
    except FileNotFoundError:
        return {"status": "error", "step": "send-goal", "detail": "docker not found"}

    return {"status": "ok", "waypoints": req.waypoints}


if STATIC_DIR.exists():
    app.mount("/assets", StaticFiles(directory=STATIC_DIR / "assets"), name="assets")

    @app.get("/")
    def index() -> FileResponse:
        return FileResponse(STATIC_DIR / "index.html")

else:

    @app.get("/")
    def index_fallback() -> dict[str, str]:
        return {
            "message": "Dashboard frontend is not built yet. Build web/ and copy dist/ to /app/static.",
        }
