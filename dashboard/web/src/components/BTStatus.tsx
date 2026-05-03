import { useEffect, useMemo, useState } from "react";
import type { BehaviorTreeLog, GoalStatusArray, NavPath } from "../types/ros";
import type { TopicState } from "../hooks/useTopic";
import { Nav2StatusBadge } from "./Nav2StatusBadge";
import { formatStamp } from "../utils/time";

type BTStatusProps = {
  message: BehaviorTreeLog | null;
  messageCount: number;
  lastUpdatedAt: number | null;
  topologyXml: string | null;
  stateSnapshotJson: string | null;
  nav2ActionStatus: TopicState<GoalStatusArray>;
  nav2Plan: TopicState<NavPath>;
};

type NodeState = {
  nodeName: string;
  status: string;
};

type FailureEvent = {
  nodeName: string;
  atMs: number;
};

type TreeNode = {
  id: string;
  label: string;
  aliases: string[];
  tagName: string;
  children: TreeNode[];
};

type ParsedTopology = {
  root: TreeNode | null;
  error: string | null;
};

type NodeKind = {
  key: "control" | "decorator" | "condition" | "action" | "outstanding";
  label: string;
};

const CONTROL_NODE_TAGS = new Set([
  "PipelineSequence",
  "RecoveryNode",
  "RoundRobinNode",
]);

const DECORATOR_NODE_TAGS = new Set([
  "DistanceController",
  "GoalUpdatedController",
  "GoalUpdater",
  "PathLongerOnApproach",
  "RateController",
  "SingleTrigger",
  "SpeedController",
]);

const CONDITION_NODE_TAGS = new Set([
  "AreErrorCodesPresent",
  "DistanceTraveledCondition",
  "GloballyUpdatedGoalCondition",
  "GoalReachedCondition",
  "GoalUpdatedCondition",
  "InitialPoseReceived",
  "IsBatteryChargingCondition",
  "IsBatteryLowCondition",
  "IsPathValidCondition",
  "IsStuckCondition",
  "PathExpiringTimerCondition",
  "TimeExpiredCondition",
  "TransformAvailableCondition",
]);

const ACTION_NODE_TAGS = new Set([
  "AssistedTeleop",
  "AssistedTeleopAction",
  "AssistedTeleopCancel",
  "BackUp",
  "BackUpAction",
  "BackUpCancel",
  "ClearCostmapAroundPose",
  "ClearCostmapAroundPoseService",
  "ClearCostmapAroundRobot",
  "ClearCostmapAroundRobotService",
  "ClearEntireCostmap",
  "ClearEntireCostmapService",
  "ComputeAndTrackRoute",
  "ComputeAndTrackRouteAction",
  "ComputePathToPose",
  "ComputePathToPoseAction",
  "ComputeRoute",
  "ComputeRouteAction",
  "ControllerCancel",
  "BtActionNode",
  "BtCancelActionNode",
  "BtServiceNode",
  "ControllerSelector",
  "DriveOnHeading",
  "DriveOnHeadingAction",
  "DriveOnHeadingCancel",
  "FollowPath",
  "FollowPathAction",
  "GetPoseFromPath",
  "GoalCheckerSelector",
  "NavigateThroughPoses",
  "NavigateThroughPosesAction",
  "NavigateToPose",
  "NavigateToPoseAction",
  "PlannerSelector",
  "ProgressCheckerSelector",
  "ReinitializeGlobalLocalization",
  "ReinitializeGlobalLocalizationService",
  "RemovePassedGoals",
  "Spin",
  "SpinAction",
  "SpinCancel",
  "SmootherSelector",
  "TruncatePath",
  "TruncatePathLocal",
  "Wait",
  "WaitAction",
  "WaitCancel",
]);

function classifyNode(tagName: string): NodeKind {
  if (CONTROL_NODE_TAGS.has(tagName)) {
    return { key: "control", label: "CONTROL" };
  }
  if (DECORATOR_NODE_TAGS.has(tagName)) {
    return { key: "decorator", label: "DECORATOR" };
  }
  if (CONDITION_NODE_TAGS.has(tagName)) {
    return { key: "condition", label: "CONDITION" };
  }
  if (ACTION_NODE_TAGS.has(tagName)) {
    return { key: "action", label: "ACTION" };
  }
  return { key: "outstanding", label: "OUTSTANDING IN ITS FIELD" };
}

function parseStateSnapshot(jsonText: string | null): Record<string, NodeState> {
  if (!jsonText || !jsonText.trim()) {
    return {};
  }

  try {
    const parsed = JSON.parse(jsonText) as {
      node_states?: Record<string, string>;
    };
    const nodeStates = parsed.node_states ?? {};
    const out: Record<string, NodeState> = {};
    for (const [nodeName, status] of Object.entries(nodeStates)) {
      if (!nodeName.trim()) {
        continue;
      }
      out[nodeName] = {
        nodeName,
        status: status || "UNKNOWN",
      };
    }
    return out;
  } catch {
    return {};
  }
}

function deriveLabel(el: Element): string {
  const tag = el.tagName;
  const name = el.getAttribute("name");
  const id = el.getAttribute("ID");

  if (name) {
    return name;
  }
  if (tag === "SubTree" && id) {
    return `SubTree:${id}`;
  }
  if (id) {
    return id;
  }
  return tag;
}

function parseTreeNode(el: Element, path: string): TreeNode {
  const label = deriveLabel(el);
  const aliases = Array.from(
    new Set([label, el.tagName, el.getAttribute("name") ?? "", el.getAttribute("ID") ?? ""]).values(),
  ).filter((item) => item.length > 0);

  const children = Array.from(el.children).map((child, index) =>
    parseTreeNode(child, `${path}/${child.tagName}[${index}]`),
  );

  return {
    id: path,
    label,
    aliases,
    tagName: el.tagName,
    children,
  };
}

function parseBtTopology(xmlText: string | null): ParsedTopology {
  if (!xmlText || !xmlText.trim()) {
    return { root: null, error: "No BT topology XML received yet from /bt_topology_xml." };
  }

  try {
    const doc = new DOMParser().parseFromString(xmlText, "application/xml");
    const parseError = doc.querySelector("parsererror");
    if (parseError) {
      return { root: null, error: "Failed to parse BT XML." };
    }

    const btRoot = doc.querySelector("root");
    if (!btRoot) {
      return { root: null, error: "Invalid BT XML: missing <root> element." };
    }

    const mainTreeId = btRoot.getAttribute("main_tree_to_execute");
    const behaviorTrees = Array.from(btRoot.getElementsByTagName("BehaviorTree"));
    if (behaviorTrees.length === 0) {
      return { root: null, error: "Invalid BT XML: missing <BehaviorTree> definitions." };
    }

    const selectedTree =
      behaviorTrees.find((tree) => tree.getAttribute("ID") === mainTreeId) ?? behaviorTrees[0];
    const selectedRootChild = selectedTree.children[0];
    if (!selectedRootChild) {
      return { root: null, error: "Selected BehaviorTree has no child nodes." };
    }

    return { root: parseTreeNode(selectedRootChild, "root"), error: null };
  } catch {
    return { root: null, error: "Unexpected error while parsing BT XML." };
  }
}

function statusForNode(node: TreeNode, statuses: Record<string, NodeState>): string {
  for (const alias of node.aliases) {
    const state = statuses[alias];
    if (state) {
      return state.status;
    }
  }
  return "UNKNOWN";
}

function renderTree(node: TreeNode, statuses: Record<string, NodeState>): JSX.Element {
  const status = statusForNode(node, statuses);
  const nodeKind = classifyNode(node.tagName);
  return (
    <li key={node.id}>
      <div className={`bt-tree-node status-${status.toLowerCase()}`}>
        <span className="bt-tree-label">{node.label}</span>
        <span className={`bt-kind kind-${nodeKind.key}`}>{nodeKind.label}</span>
        <span className="bt-status">{status}</span>
      </div>
      {node.children.length > 0 ? (
        <ul className="bt-tree-children">{node.children.map((child) => renderTree(child, statuses))}</ul>
      ) : null}
    </li>
  );
}

export function BTStatus({
  message,
  messageCount,
  lastUpdatedAt,
  topologyXml,
  stateSnapshotJson,
  nav2ActionStatus,
  nav2Plan,
}: BTStatusProps) {
  const [nodes, setNodes] = useState<Record<string, NodeState>>({});
  const [lastFailure, setLastFailure] = useState<FailureEvent | null>(null);

  useEffect(() => {
    const snapshot = parseStateSnapshot(stateSnapshotJson);
    if (Object.keys(snapshot).length === 0) {
      return;
    }

    setNodes((current) => ({
      ...snapshot,
      ...current,
    }));
  }, [stateSnapshotJson]);

  useEffect(() => {
    if (!message?.event_log?.length) {
      return;
    }

    for (const event of message.event_log ?? []) {
      if (event.current_status === "FAILURE" && event.node_name) {
        setLastFailure({
          nodeName: event.node_name,
          atMs: Date.now(),
        });
      }
    }

    setNodes((current) => {
      const next = { ...current };
      for (const event of message.event_log ?? []) {
        if (!event.node_name) {
          continue;
        }
        next[event.node_name] = {
          nodeName: event.node_name,
          status: event.current_status || "UNKNOWN",
        };
      }
      return next;
    });
  }, [message]);

  const topology = useMemo(() => parseBtTopology(topologyXml), [topologyXml]);

  return (
    <section className="panel">
      <header className="panel-header">
        <h2>Behavior Tree Status</h2>
        <Nav2StatusBadge actionStatus={nav2ActionStatus} plan={nav2Plan} />
        <div className="meta">
          <span>updates: {messageCount}</span>
          <span>last: {lastUpdatedAt ? new Date(lastUpdatedAt).toLocaleTimeString() : "--"}</span>
          <span>stamp: {formatStamp(message?.timestamp)}</span>
        </div>
      </header>
      {lastFailure ? (
        <div className="bt-failure-banner">
          Last failure: <strong>{lastFailure.nodeName}</strong> at{" "}
          {new Date(lastFailure.atMs).toLocaleTimeString()}
        </div>
      ) : null}

      <div className="bt-tree-wrap">
        {topology.root ? (
          <ul className="bt-tree">{renderTree(topology.root, nodes)}</ul>
        ) : (
          <p className="empty">{topology.error}</p>
        )}
      </div>
      {messageCount === 0 && Object.keys(nodes).length === 0 ? (
        <p className="empty">No BT transition data yet. Waiting for /behavior_tree_log.</p>
      ) : null}
    </section>
  );
}
