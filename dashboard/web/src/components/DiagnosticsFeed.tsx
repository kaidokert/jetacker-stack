import { useEffect, useRef, useState } from "react";
import type { DiagnosticArray, DiagnosticStatus } from "../types/ros";
import { formatStamp } from "../utils/time";

type DiagnosticsFeedProps = {
  message: DiagnosticArray | null;
  messageCount: number;
};

type FeedEntry = {
  id: string;
  stamp: string;
  level: number;
  name: string;
  hardwareId: string;
  message: string;
};

function levelLabel(level: number): string {
  if (level >= 2) {
    return "ERROR";
  }
  if (level === 1) {
    return "WARN";
  }
  return "OK";
}

export function DiagnosticsFeed({ message, messageCount }: DiagnosticsFeedProps) {
  const [entries, setEntries] = useState<FeedEntry[]>([]);
  const listRef = useRef<HTMLUListElement | null>(null);

  useEffect(() => {
    const statuses = message?.status ?? [];
    if (statuses.length === 0) {
      return;
    }

    const stamp = formatStamp(message?.header?.stamp);
    const filtered: DiagnosticStatus[] = statuses.filter((item) => item.level >= 1);
    if (filtered.length === 0) {
      return;
    }

    setEntries((current) => {
      const next = [...current];
      for (const item of filtered) {
        next.push({
          id: `${Date.now()}-${Math.random().toString(36).slice(2, 8)}`,
          stamp,
          level: item.level,
          name: item.name,
          hardwareId: item.hardware_id,
          message: item.message,
        });
      }
      return next.slice(-20);
    });
  }, [message]);

  useEffect(() => {
    if (!listRef.current) {
      return;
    }
    listRef.current.scrollTop = listRef.current.scrollHeight;
  }, [entries]);

  return (
    <section className="panel">
      <header className="panel-header">
        <h2>Diagnostics Feed</h2>
        <div className="meta">
          <span>msgs: {messageCount}</span>
          <span>showing WARN/ERROR (last 20)</span>
        </div>
      </header>
      {entries.length === 0 ? (
        <p className="empty">No WARN/ERROR diagnostics yet from /diagnostics.</p>
      ) : (
        <ul ref={listRef} className="diag-list">
          {entries.map((entry) => (
            <li key={entry.id} className={`diag-item level-${levelLabel(entry.level).toLowerCase()}`}>
              <span className="diag-stamp">{entry.stamp}</span>
              <span className="diag-level">{levelLabel(entry.level)}</span>
              <span className="diag-name">{entry.name}</span>
              <span className="diag-hw">{entry.hardwareId || "-"}</span>
              <span className="diag-msg">{entry.message}</span>
            </li>
          ))}
        </ul>
      )}
    </section>
  );
}
