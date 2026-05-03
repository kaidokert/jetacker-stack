"""Spawn/delete Gazebo obstacles via ros_gz_bridge services.

Used by nav2_waypoint_follower.py to schedule mid-drive obstacle drops
defined in driving_instructions yaml files.
"""

import threading
from typing import Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from ros_gz_interfaces.srv import SpawnEntity, DeleteEntity
from ros_gz_interfaces.msg import EntityFactory, Entity


# Service names — bridged in docker-compose.sim.yml ground-truth-bridge command
SPAWN_SERVICE = "/world/jetacker_world/create"
DELETE_SERVICE = "/world/jetacker_world/remove"


def build_sdf(name: str, shape: str, size: dict, mass: float = 0.5) -> str:
    """Build a minimal SDF string for the given primitive shape.

    shape: capsule | box | sphere | cylinder
    size:  dict with keys appropriate to the shape
           - capsule:  {radius, length}
           - cylinder: {radius, length}
           - sphere:   {radius}
           - box:      {x, y, z}
    """
    if shape == "capsule":
        geom = (
            f"<capsule>"
            f"<radius>{size['radius']}</radius>"
            f"<length>{size['length']}</length>"
            f"</capsule>"
        )
    elif shape == "cylinder":
        geom = (
            f"<cylinder>"
            f"<radius>{size['radius']}</radius>"
            f"<length>{size['length']}</length>"
            f"</cylinder>"
        )
    elif shape == "sphere":
        geom = f"<sphere><radius>{size['radius']}</radius></sphere>"
    elif shape == "box":
        geom = f"<box><size>{size['x']} {size['y']} {size['z']}</size></box>"
    else:
        raise ValueError(f"Unknown obstacle shape: {shape}")

    return f"""<?xml version="1.0"?>
<sdf version="1.10">
  <model name="{name}">
    <static>false</static>
    <link name="link">
      <inertial>
        <mass>{mass}</mass>
        <inertia>
          <ixx>0.01</ixx><iyy>0.01</iyy><izz>0.01</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
        </inertia>
      </inertial>
      <collision name="collision">
        <geometry>{geom}</geometry>
      </collision>
      <visual name="visual">
        <geometry>{geom}</geometry>
        <material>
          <ambient>0.8 0.2 0.2 1</ambient>
          <diffuse>0.8 0.2 0.2 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>"""


class ObstacleManager:
    """Spawn/delete obstacles via the bridged Gazebo services.

    Designed for use INSIDE a node that is already being spun by an executor.
    Service calls are async (fire-and-forget) — futures are tracked and their
    results are logged via add_done_callback so the existing spinner delivers
    them. This avoids "Executor is already spinning" errors.
    """

    def __init__(self, node: Node, timeout_sec: float = 5.0):
        self._node = node
        self._timeout = timeout_sec
        self._spawn_client = node.create_client(SpawnEntity, SPAWN_SERVICE)
        self._delete_client = node.create_client(DeleteEntity, DELETE_SERVICE)
        self._spawned: list[str] = []
        self._lock = threading.Lock()

    def wait_ready(self, timeout_sec: float = 10.0) -> bool:
        """Wait for both services to be available. Safe to call before spinning starts."""
        if not self._spawn_client.wait_for_service(timeout_sec=timeout_sec):
            self._node.get_logger().error(
                f"[obstacles] {SPAWN_SERVICE} not available after {timeout_sec}s")
            return False
        if not self._delete_client.wait_for_service(timeout_sec=timeout_sec):
            self._node.get_logger().error(
                f"[obstacles] {DELETE_SERVICE} not available after {timeout_sec}s")
            return False
        return True

    def spawn(self, spec: dict) -> bool:
        """Fire-and-forget spawn. Returns True if request was sent (not waited).

        Tracks the obstacle in `_spawned` immediately so cleanup_all() will
        also delete it even if the spawn response hasn't arrived yet.
        """
        name = spec["name"]
        sdf = build_sdf(name, spec["shape"], spec["size"], spec.get("mass", 0.5))

        req = SpawnEntity.Request()
        ef = EntityFactory()
        ef.name = name
        ef.allow_renaming = False
        ef.sdf = sdf
        pose_d = spec["pose"]
        ef.pose = Pose()
        ef.pose.position.x = float(pose_d.get("x", 0.0))
        ef.pose.position.y = float(pose_d.get("y", 0.0))
        ef.pose.position.z = float(pose_d.get("z", 0.1))
        # Optional quaternion (default upright). Useful for laying capsules sideways.
        ef.pose.orientation.x = float(pose_d.get("qx", 0.0))
        ef.pose.orientation.y = float(pose_d.get("qy", 0.0))
        ef.pose.orientation.z = float(pose_d.get("qz", 0.0))
        ef.pose.orientation.w = float(pose_d.get("qw", 1.0))
        ef.relative_to = "world"
        req.entity_factory = ef

        future = self._spawn_client.call_async(req)

        def _done_cb(fut, name=name, ef=ef):
            try:
                resp = fut.result()
                if resp is None or not resp.success:
                    self._node.get_logger().error(
                        f"[obstacles] spawn '{name}' returned failure")
                else:
                    self._node.get_logger().info(
                        f"[obstacles] spawned '{name}' at "
                        f"({ef.pose.position.x:.2f}, {ef.pose.position.y:.2f}, "
                        f"{ef.pose.position.z:.2f}) OK")
            except Exception as e:
                self._node.get_logger().error(f"[obstacles] spawn '{name}' callback error: {e}")

        future.add_done_callback(_done_cb)
        with self._lock:
            self._spawned.append(name)
        return True

    def delete(self, name: str) -> bool:
        """Fire-and-forget delete. Returns True if request was sent.

        For pre-test cleanup. Best-effort: also blocks briefly via wait_for
        on the response so subsequent operations are sequenced cleanly.
        """
        req = DeleteEntity.Request()
        ent = Entity()
        ent.name = name
        ent.type = Entity.MODEL
        req.entity = ent

        future = self._delete_client.call_async(req)

        def _done_cb(fut, name=name):
            try:
                resp = fut.result()
                success = resp is not None and resp.success
                if success:
                    self._node.get_logger().info(f"[obstacles] deleted '{name}'")
                # else: silently ignored — delete may fail because model wasn't there
            except Exception as e:
                self._node.get_logger().warn(f"[obstacles] delete '{name}' callback error: {e}")

        future.add_done_callback(_done_cb)
        with self._lock:
            if name in self._spawned:
                self._spawned.remove(name)
        return True

    def cleanup_all(self, spin_seconds: float = 2.0) -> None:
        """Issue delete requests for every obstacle this manager has spawned.

        Spins the node briefly afterwards to give the requests time to complete
        before the node is destroyed. This is the only place we spin manually —
        it is safe because cleanup is called from the finally block AFTER the
        main spin loop has exited.
        """
        with self._lock:
            names = list(self._spawned)
        for name in names:
            self.delete(name)
        if names and spin_seconds > 0:
            import time as _time
            end = _time.time() + spin_seconds
            while _time.time() < end:
                rclpy.spin_once(self._node, timeout_sec=0.1)
