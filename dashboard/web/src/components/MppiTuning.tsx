import { useCallback, useEffect, useRef, useState } from "react";
import { useSetParams, useGetParams, PARAM_TYPE, asDouble, type ParameterValue } from "../hooks/useServiceCall";
import { useTopic, type TopicState } from "../hooks/useTopic";
import { Nav2StatusBadge } from "./Nav2StatusBadge";
import type { CriticsStats, GoalStatusArray, NavPath, Twist } from "../types/ros";

// ── Critic parameter definitions ─────────────────────────────────────

type ParamDef = {
  name: string;
  type: "float" | "int" | "bool" | "float_array";
  min?: number;
  max?: number;
  step?: number;
  default: number | boolean | number[];
};

const CRITIC_DEFS: Record<string, ParamDef[]> = {
  PathFollowCritic: [
    { name: "cost_weight", type: "float", min: 0, max: 200, step: 0.5, default: 5.0 },
    { name: "cost_power", type: "int", min: 1, max: 3, default: 1 },
    { name: "offset_from_furthest", type: "int", min: 1, max: 30, default: 6 },
    { name: "threshold_to_consider", type: "float", min: 0, max: 3, step: 0.05, default: 1.4 },
  ],
  PathAlignCritic: [
    { name: "cost_weight", type: "float", min: 0, max: 200, step: 0.5, default: 5.0 },
    { name: "cost_power", type: "int", min: 1, max: 3, default: 1 },
    { name: "offset_from_furthest", type: "int", min: 1, max: 30, default: 20 },
    { name: "trajectory_point_step", type: "int", min: 1, max: 10, default: 4 },
    { name: "threshold_to_consider", type: "float", min: 0, max: 3, step: 0.05, default: 0.5 },
    { name: "use_path_orientations", type: "bool", default: false },
    { name: "max_path_occupancy_ratio", type: "float", min: 0, max: 1, step: 0.01, default: 0.07 },
  ],
  CostCritic: [
    { name: "cost_weight", type: "float", min: 0, max: 50, step: 0.5, default: 3.81 },
    { name: "cost_power", type: "int", min: 1, max: 3, default: 1 },
    { name: "consider_footprint", type: "bool", default: false },
    { name: "collision_cost", type: "float", min: 0, max: 10000000, step: 1000, default: 1000000 },
    { name: "critical_cost", type: "float", min: 0, max: 300, step: 1, default: 300 },
    { name: "near_goal_distance", type: "float", min: 0, max: 3, step: 0.1, default: 0.5 },
    { name: "trajectory_point_step", type: "int", min: 1, max: 10, default: 2 },
  ],
  ConstraintCritic: [
    { name: "cost_weight", type: "float", min: 0, max: 50, step: 0.5, default: 4.0 },
    { name: "cost_power", type: "int", min: 1, max: 3, default: 1 },
  ],
  GoalCritic: [
    { name: "cost_weight", type: "float", min: 0, max: 50, step: 0.5, default: 5.0 },
    { name: "cost_power", type: "int", min: 1, max: 3, default: 1 },
    { name: "threshold_to_consider", type: "float", min: 0, max: 3, step: 0.05, default: 1.4 },
  ],
  GoalAngleCritic: [
    { name: "cost_weight", type: "float", min: 0, max: 50, step: 0.5, default: 3.0 },
    { name: "cost_power", type: "int", min: 1, max: 3, default: 1 },
    { name: "threshold_to_consider", type: "float", min: 0, max: 3, step: 0.05, default: 0.5 },
  ],
  PathAngleCritic: [
    { name: "cost_weight", type: "float", min: 0, max: 50, step: 0.5, default: 2.0 },
    { name: "cost_power", type: "int", min: 1, max: 3, default: 1 },
    { name: "threshold_to_consider", type: "float", min: 0, max: 3, step: 0.05, default: 0.5 },
    { name: "offset_from_furthest", type: "int", min: 1, max: 30, default: 4 },
    { name: "mode", type: "int", min: 0, max: 2, default: 0 },
  ],
  PreferForwardCritic: [
    { name: "cost_weight", type: "float", min: 0, max: 50, step: 0.5, default: 5.0 },
    { name: "cost_power", type: "int", min: 1, max: 3, default: 1 },
    { name: "threshold_to_consider", type: "float", min: 0, max: 3, step: 0.05, default: 0.5 },
  ],
  VelocityDeadbandCritic: [
    { name: "cost_weight", type: "float", min: 0, max: 50, step: 0.5, default: 35.0 },
    { name: "cost_power", type: "int", min: 1, max: 3, default: 1 },
    { name: "deadband_velocities", type: "float_array", default: [0.05, 0.0, 0.0] },
  ],
};

const GLOBAL_PARAMS: ParamDef[] = [
  { name: "vx_std", type: "float", min: 0.01, max: 1.0, step: 0.01, default: 0.2 },
  { name: "wz_std", type: "float", min: 0.01, max: 2.0, step: 0.01, default: 0.4 },
  { name: "temperature", type: "float", min: 0.01, max: 2.0, step: 0.005, default: 0.3 },
  { name: "gamma", type: "float", min: 0.001, max: 0.1, step: 0.001, default: 0.015 },
  { name: "batch_size", type: "int", min: 100, max: 5000, step: 100, default: 1000 },
  { name: "iteration_count", type: "int", min: 1, max: 10, default: 1 },
  { name: "time_steps", type: "int", min: 10, max: 120, default: 56 },
  { name: "AckermannConstraints.min_turning_r", type: "float", min: 0.1, max: 2.0, step: 0.05, default: 0.35 },
  { name: "enforce_path_inversion", type: "bool", default: true },
  { name: "inversion_xy_tolerance", type: "float", min: 0.05, max: 1.0, step: 0.05, default: 0.2 },
  { name: "inversion_yaw_tolerance", type: "float", min: 0.05, max: 1.5, step: 0.05, default: 0.3 },
  { name: "max_robot_pose_search_dist", type: "float", min: 0.1, max: 5.0, step: 0.1, default: 1.5 },
];

const PLANNER_PARAMS: ParamDef[] = [
  { name: "minimum_turning_radius", type: "float", min: 0.1, max: 3.0, step: 0.05, default: 0.8 },
  { name: "reverse_penalty", type: "float", min: 1.0, max: 10.0, step: 0.5, default: 2.0 },
  { name: "change_penalty", type: "float", min: 0.0, max: 10.0, step: 0.1, default: 1.0 },
  { name: "smooth_path", type: "bool", default: true },
  { name: "angle_quantization_bins", type: "int", min: 36, max: 360, step: 1, default: 72 },
  { name: "tolerance", type: "float", min: 0.05, max: 1.0, step: 0.05, default: 0.25 },
];

const CRITIC_NAMES = Object.keys(CRITIC_DEFS);
const NODE = "controller_server";
const PLANNER_NODE = "planner_server";
const PREFIX = "FollowPath";
const PLANNER_PREFIX = "GridBased";

// ── Helpers ──────────────────────────────────────────────────────────

function paramValueToJS(pv: ParameterValue | undefined | null): boolean | number | string | number[] | null {
  if (!pv || typeof pv !== "object" || !("type" in pv)) return null;
  switch (pv.type) {
    case PARAM_TYPE.BOOL:
      return pv.bool_value ?? false;
    case PARAM_TYPE.INTEGER:
      return pv.integer_value ?? 0;
    case PARAM_TYPE.DOUBLE:
      return pv.double_value ?? 0;
    case PARAM_TYPE.STRING:
      return pv.string_value ?? "";
    case PARAM_TYPE.DOUBLE_ARRAY:
      return pv.double_array_value ?? [];
    default:
      return null;
  }
}

/** Wrap a numeric value with the correct ROS type based on ParamDef. */
function typedValue(val: number, def: ParamDef) {
  return def.type === "float" ? asDouble(val) : val;
}

// ── Component ────────────────────────────────────────────────────────

type Props = {
  socket: WebSocket | null;
  nav2ActionStatus: TopicState<GoalStatusArray>;
  nav2Plan: TopicState<NavPath>;
};

export function MppiTuning({ socket, nav2ActionStatus, nav2Plan }: Props) {
  const setParams = useSetParams(socket, NODE);
  const getParams = useGetParams(socket, NODE);
  const setPlannerParams = useSetParams(socket, PLANNER_NODE, 30000);
  const getPlannerParams = useGetParams(socket, PLANNER_NODE, 30000);
  const setHwBridgeParams = useSetParams(socket, "gazebo_hardware_bridge");
  const getHwBridgeParams = useGetParams(socket, "gazebo_hardware_bridge");

  // Local state: critic values and enabled flags
  const [criticValues, setCriticValues] = useState<Record<string, Record<string, unknown>>>({});
  const [globalValues, setGlobalValues] = useState<Record<string, number>>({});
  const [plannerValues, setPlannerValues] = useState<Record<string, number>>({});
  const [loading, setLoading] = useState(false);
  const [retrying, setRetrying] = useState(false);
  const [teleporting, setTeleporting] = useState(false);
  const [frozen, setFrozen] = useState<boolean | null>(null);
  const [status, setStatus] = useState<string | null>(null);
  const [waypointsInput, setWaypointsInput] = useState("nav2_matrix_3_forward_left_90");

  // Live topics
  const criticsStats = useTopic<CriticsStats>(
    socket,
    "/controller_server/critics_stats",
    "nav2_critics_msgs/msg/CriticsStats",
  );
  const cmdVel = useTopic<Twist>(socket, "/cmd_vel", "geometry_msgs/msg/Twist");

  // ── Fetch current params ──────────────────────────────────────────

  const fetchParams = useCallback(async () => {
    if (!socket || socket.readyState !== WebSocket.OPEN) return;
    setLoading(true);
    setStatus(null);
    const errors: string[] = [];

    // Fetch global params
    try {
      const globalNames = GLOBAL_PARAMS.map((p) => `${PREFIX}.${p.name}`);
      const values = await getParams(globalNames);
      console.log("[MPPI] global raw:", globalNames, values);
      const gv: Record<string, number> = {};
      for (let i = 0; i < GLOBAL_PARAMS.length; i++) {
        const val = paramValueToJS(values[i]);
        if (typeof val === "number") {
          gv[GLOBAL_PARAMS[i].name] = val;
        }
      }
      setGlobalValues(gv);
    } catch (e) {
      errors.push(`global: ${e instanceof Error ? e.message : String(e)}`);
    }

    // Fetch planner params via rosbridge
    try {
      const plannerNames = PLANNER_PARAMS.map((p) => `${PLANNER_PREFIX}.${p.name}`);
      const pValues = await getPlannerParams(plannerNames);
      console.log("[MPPI] planner raw:", plannerNames, pValues);
      const pv: Record<string, number> = {};
      for (let i = 0; i < PLANNER_PARAMS.length; i++) {
        const val = paramValueToJS(pValues[i]);
        if (typeof val === "number") {
          pv[PLANNER_PARAMS[i].name] = val;
        }
      }
      setPlannerValues(pv);
    } catch (e) {
      errors.push(`planner: ${e instanceof Error ? e.message : String(e)}`);
    }

    // Fetch each critic separately so one failure doesn't break all
    const cv: Record<string, Record<string, unknown>> = {};
    for (const critic of CRITIC_NAMES) {
      try {
        const names = [
          `${PREFIX}.${critic}.enabled`,
          ...CRITIC_DEFS[critic].map((p) => `${PREFIX}.${critic}.${p.name}`),
        ];
        const values = await getParams(names);
        console.log(`[MPPI] ${critic} raw:`, names, values);
        cv[critic] = {};
        cv[critic].enabled = paramValueToJS(values[0]);
        for (let j = 0; j < CRITIC_DEFS[critic].length; j++) {
          const v = paramValueToJS(values[j + 1]);
          if (v !== null) cv[critic][CRITIC_DEFS[critic][j].name] = v;
        }
      } catch (e) {
        errors.push(`${critic}: ${e instanceof Error ? e.message : String(e)}`);
        cv[critic] = { enabled: false };
      }
    }
    setCriticValues(cv);

    if (errors.length > 0) {
      setStatus(`Loaded with errors: ${errors.join("; ")}`);
    } else {
      setStatus("Loaded");
    }
    setLoading(false);
  }, [socket, getParams, getPlannerParams]);

  // Auto-fetch on connection
  const hasFetched = useRef(false);
  useEffect(() => {
    if (socket && socket.readyState === WebSocket.OPEN && !hasFetched.current) {
      hasFetched.current = true;
      fetchParams();
    }
    if (!socket || socket.readyState !== WebSocket.OPEN) {
      hasFetched.current = false;
    }
  }, [socket, fetchParams]);

  // ── Apply changes ──────────────────────────────────────────────────

  const applyGlobal = useCallback(async () => {
    setStatus(null);
    try {
      const params: Record<string, ReturnType<typeof typedValue> | boolean> = {};
      for (const p of GLOBAL_PARAMS) {
        if (globalValues[p.name] !== undefined) {
          if (p.type === "bool") {
            params[`${PREFIX}.${p.name}`] = !!globalValues[p.name];
          } else {
            params[`${PREFIX}.${p.name}`] = typedValue(globalValues[p.name], p);
          }
        }
      }
      await setParams(params);
      setStatus("Global params applied");
    } catch (e) {
      setStatus(`Error: ${e instanceof Error ? e.message : String(e)}`);
    }
  }, [globalValues, setParams]);

  const applyPlanner = useCallback(async () => {
    setStatus(null);
    try {
      const params: Record<string, ReturnType<typeof typedValue> | boolean> = {};
      for (const p of PLANNER_PARAMS) {
        if (plannerValues[p.name] !== undefined) {
          if (p.type === "bool") {
            params[`${PLANNER_PREFIX}.${p.name}`] = !!plannerValues[p.name];
          } else {
            params[`${PLANNER_PREFIX}.${p.name}`] = typedValue(plannerValues[p.name], p);
          }
        }
      }
      await setPlannerParams(params);
      setStatus("Planner params applied");
    } catch (e) {
      setStatus(`Error: ${e instanceof Error ? e.message : String(e)}`);
    }
  }, [plannerValues, setPlannerParams]);

  const toggleCritic = useCallback(
    async (critic: string) => {
      const current = criticValues[critic]?.enabled ?? false;
      const newVal = !current;
      try {
        await setParams({ [`${PREFIX}.${critic}.enabled`]: newVal });
        setCriticValues((prev) => ({
          ...prev,
          [critic]: { ...prev[critic], enabled: newVal },
        }));
        setStatus(`${critic} ${newVal ? "enabled" : "disabled"}`);
      } catch (e) {
        setStatus(`Error toggling ${critic}: ${e instanceof Error ? e.message : String(e)}`);
      }
    },
    [criticValues, setParams],
  );

  const applyCriticParams = useCallback(
    async (critic: string) => {
      const cv = criticValues[critic];
      if (!cv) return;
      setStatus(null);
      try {
        const params: Record<string, unknown> = {};
        for (const p of CRITIC_DEFS[critic]) {
          if (cv[p.name] !== undefined) {
            const val = cv[p.name];
            params[`${PREFIX}.${critic}.${p.name}`] =
              p.type === "float" && typeof val === "number" ? asDouble(val) : val;
          }
        }
        await setParams(params as Record<string, boolean | number | string | number[]>);
        setStatus(`${critic} params applied`);
      } catch (e) {
        setStatus(`Error: ${e instanceof Error ? e.message : String(e)}`);
      }
    },
    [criticValues, setParams],
  );

  // ── Retry ──────────────────────────────────────────────────────────

  const retry = useCallback(async () => {
    setRetrying(true);
    setStatus(null);
    try {
      const resp = await fetch("/api/mppi/retry", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ waypoints: waypointsInput }),
      });
      const data = await resp.json();
      setStatus(data.status === "ok" ? `Retry sent: ${data.waypoints}` : `Retry error: ${data.detail}`);
    } catch (e) {
      setStatus(`Retry failed: ${e instanceof Error ? e.message : String(e)}`);
    } finally {
      setRetrying(false);
    }
  }, [waypointsInput]);

  // ── Teleport ────────────────────────────────────────────────────────

  const teleport = useCallback(async () => {
    setTeleporting(true);
    setStatus(null);
    try {
      const resp = await fetch("/api/mppi/teleport", { method: "POST" });
      const data = await resp.json();
      setStatus(data.success ? data.message : `Teleport error: ${data.message}`);
    } catch (e) {
      setStatus(`Teleport failed: ${e instanceof Error ? e.message : String(e)}`);
    } finally {
      setTeleporting(false);
    }
  }, []);

  // ── Freeze (immobilize robot) ────────────────────────────────────

  const setClutch = useCallback(async (engage: boolean) => {
    const prev = frozen;
    setFrozen(!engage); // optimistic
    setStatus(null);
    try {
      await setHwBridgeParams({ clutch: engage });
      const label = engage ? "engaged — robot free" : "disengaged — robot stopped";
      setStatus(`Clutch ${label}`);
    } catch (e) {
      setFrozen(prev); // revert
      setStatus(`Clutch failed: ${e instanceof Error ? e.message : String(e)}`);
    }
  }, [frozen, setHwBridgeParams]);

  // Check clutch status on mount via rosbridge
  useEffect(() => {
    if (!socket || socket.readyState !== WebSocket.OPEN) return;
    getHwBridgeParams(["clutch"]).then((values) => {
      const val = paramValueToJS(values[0]);
      if (typeof val === "boolean") {
        setFrozen(!val); // clutch=true means NOT frozen
      }
    }).catch(() => {});
  }, [socket, getHwBridgeParams]);

  // ── Live data ──────────────────────────────────────────────────────

  const stats = criticsStats.message;
  const vel = cmdVel.message;
  const maxCost = stats ? Math.max(1, ...stats.costs_best.map(Math.abs)) : 1;

  // ── Render ─────────────────────────────────────────────────────────

  return (
    <div className="mppi-tuning">
      {/* Status bar */}
      <div className="mppi-status-bar">
        <Nav2StatusBadge actionStatus={nav2ActionStatus} plan={nav2Plan} />
        <button className="mppi-btn" onClick={fetchParams} disabled={loading}>
          {loading ? "Loading..." : "Refresh"}
        </button>
        <button className="mppi-btn mppi-btn-teleport" onClick={teleport} disabled={teleporting}>
          {teleporting ? "Teleporting..." : "Reset Position"}
        </button>
        <label className={`mppi-clutch ${frozen ? "mppi-clutch-off" : ""}`}>
          <span className="mppi-clutch-label">Clutch</span>
          <label className="mppi-toggle">
            <input type="checkbox" checked={!frozen} onChange={(e) => setClutch(e.target.checked)} />
            <span className="mppi-toggle-slider" />
          </label>
        </label>
        <div className="mppi-retry-group">
          <input
            className="mppi-waypoints-input"
            value={waypointsInput}
            onChange={(e) => setWaypointsInput(e.target.value)}
          />
          <button className="mppi-btn mppi-btn-retry" onClick={retry} disabled={retrying}>
            {retrying ? "Retrying..." : "Retry"}
          </button>
        </div>
        {status && <span className="mppi-status-msg">{status}</span>}
      </div>

      {/* Global MPPI params */}
      <div className="mppi-global panel">
        <div className="panel-header">
          <h2>MPPI Global</h2>
          <button className="mppi-btn" onClick={applyGlobal}>
            Apply
          </button>
        </div>
        <div className="mppi-global-grid">
          {GLOBAL_PARAMS.map((p) => (
            <div key={p.name} className="mppi-param-row">
              <label className="mppi-param-label">{p.name}</label>
              {p.type === "bool" ? (
                <label className="mppi-toggle">
                  <input
                    type="checkbox"
                    checked={(globalValues[p.name] as unknown as boolean) ?? (p.default as boolean)}
                    onChange={(e) =>
                      setGlobalValues((prev) => ({
                        ...prev,
                        [p.name]: e.target.checked ? 1 : 0,
                      }))
                    }
                  />
                  <span className="mppi-toggle-slider" />
                </label>
              ) : (
                <>
                  <input
                    type="range"
                    className="mppi-slider"
                    min={p.min}
                    max={p.max}
                    step={p.step ?? 1}
                    value={globalValues[p.name] ?? p.default}
                    onChange={(e) =>
                      setGlobalValues((prev) => ({
                        ...prev,
                        [p.name]: p.type === "int" ? parseInt(e.target.value) : parseFloat(e.target.value),
                      }))
                    }
                  />
                  <input
                    type="number"
                    className="mppi-number"
                    step={p.step ?? 1}
                    value={globalValues[p.name] ?? p.default}
                    onChange={(e) =>
                      setGlobalValues((prev) => ({
                        ...prev,
                        [p.name]: p.type === "int" ? parseInt(e.target.value) : parseFloat(e.target.value),
                      }))
                    }
                  />
                </>
              )}
            </div>
          ))}
        </div>
      </div>

      {/* Planner params */}
      <div className="mppi-global panel">
        <div className="panel-header">
          <h2>Planner (SmacHybrid)</h2>
          <button className="mppi-btn" onClick={applyPlanner}>
            Apply
          </button>
        </div>
        <div className="mppi-global-grid">
          {PLANNER_PARAMS.map((p) => (
            <div key={p.name} className="mppi-param-row">
              <label className="mppi-param-label">{p.name}</label>
              {p.type === "bool" ? (
                <label className="mppi-toggle">
                  <input
                    type="checkbox"
                    checked={(plannerValues[p.name] as unknown as boolean) ?? (p.default as boolean)}
                    onChange={(e) =>
                      setPlannerValues((prev) => ({
                        ...prev,
                        [p.name]: e.target.checked ? 1 : 0,
                      }))
                    }
                  />
                  <span className="mppi-toggle-slider" />
                </label>
              ) : (
                <>
                  <input
                    type="range"
                    className="mppi-slider"
                    min={p.min}
                    max={p.max}
                    step={p.step ?? 1}
                    value={plannerValues[p.name] ?? p.default}
                    onChange={(e) =>
                      setPlannerValues((prev) => ({
                        ...prev,
                        [p.name]: p.type === "int" ? parseInt(e.target.value) : parseFloat(e.target.value),
                      }))
                    }
                  />
                  <input
                    type="number"
                    className="mppi-number"
                    step={p.step ?? 1}
                    value={plannerValues[p.name] ?? p.default}
                    onChange={(e) =>
                      setPlannerValues((prev) => ({
                        ...prev,
                        [p.name]: p.type === "int" ? parseInt(e.target.value) : parseFloat(e.target.value),
                      }))
                    }
                  />
                </>
              )}
            </div>
          ))}
        </div>
      </div>

      {/* Critic cards */}
      <div className="mppi-critics-grid">
        {CRITIC_NAMES.map((critic) => {
          const cv = criticValues[critic] ?? {};
          const enabled = cv.enabled === true;
          return (
            <div key={critic} className={`mppi-critic-card panel ${enabled ? "critic-enabled" : "critic-disabled"}`}>
              <div className="mppi-critic-header">
                <h3>{critic.replace("Critic", "")}</h3>
                <label className="mppi-toggle">
                  <input type="checkbox" checked={enabled} onChange={() => toggleCritic(critic)} />
                  <span className="mppi-toggle-slider" />
                </label>
              </div>
              {enabled && (
                <>
                  <div className="mppi-critic-params">
                    {CRITIC_DEFS[critic].map((p) => {
                      if (p.type === "bool") {
                        return (
                          <div key={p.name} className="mppi-param-row">
                            <label className="mppi-param-label">{p.name}</label>
                            <label className="mppi-toggle mppi-toggle-sm">
                              <input
                                type="checkbox"
                                checked={(cv[p.name] as boolean) ?? (p.default as boolean)}
                                onChange={(e) =>
                                  setCriticValues((prev) => ({
                                    ...prev,
                                    [critic]: { ...prev[critic], [p.name]: e.target.checked },
                                  }))
                                }
                              />
                              <span className="mppi-toggle-slider" />
                            </label>
                          </div>
                        );
                      }
                      if (p.type === "float_array") {
                        const arr = (cv[p.name] as number[]) ?? (p.default as number[]);
                        return (
                          <div key={p.name} className="mppi-param-row">
                            <label className="mppi-param-label">{p.name}</label>
                            <input
                              type="text"
                              className="mppi-number mppi-array-input"
                              value={arr.join(", ")}
                              onChange={(e) => {
                                const vals = e.target.value.split(",").map((s) => parseFloat(s.trim()));
                                if (vals.every((v) => !isNaN(v))) {
                                  setCriticValues((prev) => ({
                                    ...prev,
                                    [critic]: { ...prev[critic], [p.name]: vals },
                                  }));
                                }
                              }}
                            />
                          </div>
                        );
                      }
                      const val = (cv[p.name] as number) ?? (p.default as number);
                      return (
                        <div key={p.name} className="mppi-param-row">
                          <label className="mppi-param-label">{p.name}</label>
                          <input
                            type="range"
                            className="mppi-slider"
                            min={p.min}
                            max={p.max}
                            step={p.step ?? 1}
                            value={val}
                            onChange={(e) =>
                              setCriticValues((prev) => ({
                                ...prev,
                                [critic]: {
                                  ...prev[critic],
                                  [p.name]:
                                    p.type === "int" ? parseInt(e.target.value) : parseFloat(e.target.value),
                                },
                              }))
                            }
                          />
                          <input
                            type="number"
                            className="mppi-number"
                            step={p.step ?? 1}
                            value={val}
                            onChange={(e) =>
                              setCriticValues((prev) => ({
                                ...prev,
                                [critic]: {
                                  ...prev[critic],
                                  [p.name]:
                                    p.type === "int" ? parseInt(e.target.value) : parseFloat(e.target.value),
                                },
                              }))
                            }
                          />
                        </div>
                      );
                    })}
                  </div>
                  <button className="mppi-btn mppi-btn-sm" onClick={() => applyCriticParams(critic)}>
                    Apply
                  </button>
                </>
              )}
            </div>
          );
        })}
      </div>

      {/* Live data */}
      <div className="mppi-live panel">
        <div className="panel-header">
          <h2>Live Data</h2>
          <span className="meta">
            critics: {criticsStats.messageCount} msgs | cmd_vel: {cmdVel.messageCount} msgs
          </span>
        </div>

        {/* cmd_vel readout */}
        <div className="mppi-cmdvel">
          <span>
            vx: <strong>{vel?.linear.x.toFixed(3) ?? "—"}</strong>
          </span>
          <span>
            wz: <strong>{vel?.angular.z.toFixed(3) ?? "—"}</strong>
          </span>
        </div>

        {/* Critic cost bars */}
        {stats ? (
          <div className="mppi-cost-bars">
            {stats.critic_names.map((name, i) => {
              const cost = stats.costs_best[i] ?? 0;
              const pct = Math.min(100, (Math.abs(cost) / maxCost) * 100);
              return (
                <div key={name} className="mppi-cost-row">
                  <span className="mppi-cost-label">{name.replace("Critic", "")}</span>
                  <div className="mppi-cost-bar-bg">
                    <div
                      className="mppi-cost-bar-fill"
                      style={{ width: `${pct}%` }}
                    />
                  </div>
                  <span className="mppi-cost-value">{cost.toFixed(1)}</span>
                </div>
              );
            })}
          </div>
        ) : (
          <p className="empty">No critics_stats data yet</p>
        )}
      </div>
    </div>
  );
}
