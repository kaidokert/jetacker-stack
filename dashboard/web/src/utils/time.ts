import type { RosTime } from "../types/ros";

export function rosTimeToMillis(stamp?: RosTime): number | null {
  if (!stamp) {
    return null;
  }

  const sec = stamp.sec ?? stamp.secs ?? 0;
  const nsec = stamp.nanosec ?? stamp.nsecs ?? 0;

  if (!Number.isFinite(sec) || !Number.isFinite(nsec)) {
    return null;
  }

  return sec * 1_000 + Math.floor(nsec / 1_000_000);
}

export function formatStamp(stamp?: RosTime): string {
  const ms = rosTimeToMillis(stamp);
  if (ms === null) {
    return "--";
  }
  return new Date(ms).toLocaleTimeString();
}
