import { useEffect, useMemo, useState } from "react";

type RosBridgeMessage<T> = {
  op: string;
  topic?: string;
  msg?: T;
};

export type TopicState<T> = {
  message: T | null;
  messageCount: number;
  lastUpdatedAt: number | null;
};

function buildSubscriptionId(topic: string): string {
  return `sub:${topic}:${Math.random().toString(36).slice(2, 8)}`;
}

export function useTopic<T>(
  socket: WebSocket | null,
  topic: string,
  type: string,
): TopicState<T> {
  const [message, setMessage] = useState<T | null>(null);
  const [messageCount, setMessageCount] = useState(0);
  const [lastUpdatedAt, setLastUpdatedAt] = useState<number | null>(null);

  useEffect(() => {
    if (!socket || socket.readyState !== WebSocket.OPEN) {
      return;
    }

    const id = buildSubscriptionId(topic);
    const subscribePayload = {
      op: "subscribe",
      id,
      topic,
      type,
      queue_length: 1,
    };
    socket.send(JSON.stringify(subscribePayload));

    const onMessage = (event: MessageEvent) => {
      try {
        const parsed = JSON.parse(event.data) as RosBridgeMessage<T>;
        if (parsed.op !== "publish" || parsed.topic !== topic || !parsed.msg) {
          return;
        }
        setMessage(parsed.msg);
        setMessageCount((current) => current + 1);
        setLastUpdatedAt(Date.now());
      } catch {
        // Ignore malformed messages from unrelated topics.
      }
    };

    socket.addEventListener("message", onMessage);

    return () => {
      socket.removeEventListener("message", onMessage);
      if (socket.readyState === WebSocket.OPEN) {
        socket.send(JSON.stringify({ op: "unsubscribe", id, topic }));
      }
    };
  }, [socket, topic, type]);

  return useMemo(
    () => ({
      message,
      messageCount,
      lastUpdatedAt,
    }),
    [lastUpdatedAt, message, messageCount],
  );
}
