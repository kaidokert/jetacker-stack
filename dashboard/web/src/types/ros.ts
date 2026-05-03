export type RosTime = {
  sec?: number;
  nanosec?: number;
  secs?: number;
  nsecs?: number;
};

export type BehaviorTreeStatusChange = {
  node_name: string;
  current_status: string;
  previous_status?: string;
  timestamp?: RosTime;
};

export type BehaviorTreeLog = {
  timestamp?: RosTime;
  event_log?: BehaviorTreeStatusChange[];
};

export type DiagnosticKeyValue = {
  key: string;
  value: string;
};

export type DiagnosticStatus = {
  level: number;
  name: string;
  message: string;
  hardware_id: string;
  values?: DiagnosticKeyValue[];
};

export type DiagnosticArray = {
  header?: {
    stamp?: RosTime;
  };
  status?: DiagnosticStatus[];
};

export type StringMsg = {
  data: string;
};

export type BTNodeStateSnapshot = {
  updated_at_unix?: number;
  node_states?: Record<string, string>;
};

/** geometry_msgs/msg/Twist */
export type Twist = {
  linear: { x: number; y: number; z: number };
  angular: { x: number; y: number; z: number };
};

/** nav2_critics_msgs/msg/CriticsStats (backport overlay) */
export type CriticsStats = {
  critic_names: string[];
  costs_best: number[];
  costs_mean: number[];
};

/** action_msgs/msg/GoalStatus */
export type GoalStatus = {
  goal_info: {
    goal_id: { uuid: number[] };
    stamp: RosTime;
  };
  status: number;
};

/** action_msgs/msg/GoalStatusArray */
export type GoalStatusArray = {
  status_list: GoalStatus[];
};

/** Status constants matching action_msgs/msg/GoalStatus */
export const NAV2_STATUS = {
  UNKNOWN: 0,
  ACCEPTED: 1,
  EXECUTING: 2,
  CANCELING: 3,
  SUCCEEDED: 4,
  CANCELED: 5,
  ABORTED: 6,
} as const;

/** nav_msgs/msg/Path (only header needed for timing) */
export type NavPath = {
  header: { stamp: RosTime; frame_id: string };
  poses: unknown[];
};
