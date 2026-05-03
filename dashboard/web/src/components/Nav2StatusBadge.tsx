import { useMemo } from "react";
import { NAV2_STATUS, type GoalStatusArray, type NavPath } from "../types/ros";
import type { TopicState } from "../hooks/useTopic";

type Nav2Phase = "idle" | "planning" | "navigating" | "succeeded" | "aborted" | "canceled";

const PHASE_LABELS: Record<Nav2Phase, string> = {
  idle: "Idle",
  planning: "Planning...",
  navigating: "Navigating",
  succeeded: "Succeeded",
  aborted: "Aborted",
  canceled: "Canceled",
};

type Props = {
  actionStatus: TopicState<GoalStatusArray>;
  plan: TopicState<NavPath>;
};

export function Nav2StatusBadge({ actionStatus, plan }: Props) {
  const phase = useMemo((): Nav2Phase => {
    const msg = actionStatus.message;
    if (!msg || msg.status_list.length === 0) return "idle";

    // Find the most recent goal (last in the list)
    const latest = msg.status_list[msg.status_list.length - 1];
    switch (latest.status) {
      case NAV2_STATUS.ACCEPTED:
        return "planning";
      case NAV2_STATUS.EXECUTING: {
        // Distinguish planning vs navigating:
        // If the plan topic was updated AFTER the action status started, planner is done.
        const planTs = plan.lastUpdatedAt ?? 0;
        const goalTs = actionStatus.lastUpdatedAt ?? 0;
        // If we've received a plan since the goal became active, we're navigating.
        // Otherwise still planning (or planner is recomputing).
        return planTs > 0 && planTs >= goalTs - 2000 ? "navigating" : "planning";
      }
      case NAV2_STATUS.CANCELING:
        return "canceled";
      case NAV2_STATUS.SUCCEEDED:
        return "succeeded";
      case NAV2_STATUS.ABORTED:
        return "aborted";
      case NAV2_STATUS.CANCELED:
        return "canceled";
      default:
        return "idle";
    }
  }, [actionStatus.message, actionStatus.lastUpdatedAt, plan.lastUpdatedAt]);

  const label = PHASE_LABELS[phase];

  return <span className={`nav2-badge nav2-badge-${phase}`}>{label}</span>;
}
