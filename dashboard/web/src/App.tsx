import { useState } from "react";
import { BTStatus } from "./components/BTStatus";
import { DiagnosticsFeed } from "./components/DiagnosticsFeed";
import { MppiTuning } from "./components/MppiTuning";
import { useRosBridge } from "./hooks/useRosBridge";
import { useTopic } from "./hooks/useTopic";
import type { BehaviorTreeLog, DiagnosticArray, GoalStatusArray, NavPath, StringMsg } from "./types/ros";

function resolveRosbridgeUrl(): string {
  if (import.meta.env.VITE_ROSBRIDGE_URL) {
    return import.meta.env.VITE_ROSBRIDGE_URL as string;
  }

  const protocol = window.location.protocol === "https:" ? "wss" : "ws";
  return `${protocol}://${window.location.hostname}:9090`;
}

function statusClass(status: string): string {
  if (status === "open") {
    return "open";
  }
  if (status === "connecting") {
    return "connecting";
  }
  return "closed";
}

type Tab = "overview" | "mppi";

export function App() {
  const [activeTab, setActiveTab] = useState<Tab>("overview");
  const rosbridgeUrl = resolveRosbridgeUrl();
  const connection = useRosBridge(rosbridgeUrl);
  const btLogGlobal = useTopic<BehaviorTreeLog>(
    connection.socket,
    "/behavior_tree_log",
    "nav2_msgs/msg/BehaviorTreeLog",
  );
  const btLogNamespaced = useTopic<BehaviorTreeLog>(
    connection.socket,
    "/bt_navigator/behavior_tree_log",
    "nav2_msgs/msg/BehaviorTreeLog",
  );
  const diagnostics = useTopic<DiagnosticArray>(
    connection.socket,
    "/diagnostics",
    "diagnostic_msgs/msg/DiagnosticArray",
  );
  const btTopology = useTopic<StringMsg>(connection.socket, "/bt_topology_xml", "std_msgs/msg/String");
  const btStateSnapshot = useTopic<StringMsg>(
    connection.socket,
    "/bt_node_state_snapshot",
    "std_msgs/msg/String",
  );
  const nav2ActionStatus = useTopic<GoalStatusArray>(
    connection.socket,
    "/navigate_to_pose/_action/status",
    "action_msgs/msg/GoalStatusArray",
  );
  const nav2Plan = useTopic<NavPath>(
    connection.socket,
    "/plan",
    "nav_msgs/msg/Path",
  );
  const btLog =
    btLogGlobal.messageCount > 0 ||
    (btLogGlobal.message !== null && btLogNamespaced.messageCount === 0)
      ? btLogGlobal
      : btLogNamespaced;

  return (
    <main className="page">
      <header className="topbar">
        <h1>Robot Dashboard</h1>
        <div className="connection">
          <span className={`dot ${statusClass(connection.status)}`} />
          <span>{connection.status}</span>
          <code>{connection.url}</code>
          {connection.error ? <span className="error">{connection.error}</span> : null}
        </div>
      </header>

      <nav className="tab-bar">
        <button className={`tab-btn ${activeTab === "overview" ? "active" : ""}`} onClick={() => setActiveTab("overview")}>
          Overview
        </button>
        <button className={`tab-btn ${activeTab === "mppi" ? "active" : ""}`} onClick={() => setActiveTab("mppi")}>
          MPPI Tuning
        </button>
      </nav>

      {activeTab === "overview" && (
        <section className="grid">
          <BTStatus
            message={btLog.message}
            messageCount={btLog.messageCount}
            lastUpdatedAt={btLog.lastUpdatedAt}
            topologyXml={btTopology.message?.data ?? null}
            stateSnapshotJson={btStateSnapshot.message?.data ?? null}
            nav2ActionStatus={nav2ActionStatus}
            nav2Plan={nav2Plan}
          />
          <DiagnosticsFeed message={diagnostics.message} messageCount={diagnostics.messageCount} />
        </section>
      )}

      {activeTab === "mppi" && (
        <MppiTuning socket={connection.socket} nav2ActionStatus={nav2ActionStatus} nav2Plan={nav2Plan} />
      )}
    </main>
  );
}
