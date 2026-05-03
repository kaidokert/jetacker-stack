import { useEffect, useMemo, useRef, useState } from "react";

export type ConnectionStatus = "connecting" | "open" | "closed" | "error";

export type RosBridgeConnection = {
  socket: WebSocket | null;
  status: ConnectionStatus;
  error: string | null;
  url: string;
};

const RECONNECT_DELAY_MS = 2000;

export function useRosBridge(url: string): RosBridgeConnection {
  const [status, setStatus] = useState<ConnectionStatus>("connecting");
  const [error, setError] = useState<string | null>(null);
  const [socket, setSocket] = useState<WebSocket | null>(null);
  const retryTimer = useRef<number | null>(null);

  useEffect(() => {
    let active = true;
    let ws: WebSocket | null = null;

    const connect = () => {
      if (!active) {
        return;
      }

      setStatus("connecting");
      setError(null);
      ws = new WebSocket(url);

      ws.onopen = () => {
        if (!active) {
          return;
        }
        setSocket(ws);
        setStatus("open");
      };

      ws.onerror = () => {
        if (!active) {
          return;
        }
        setStatus("error");
        setError("Failed to connect to rosbridge.");
      };

      ws.onclose = () => {
        if (!active) {
          return;
        }
        setSocket(null);
        setStatus("closed");
        retryTimer.current = window.setTimeout(connect, RECONNECT_DELAY_MS);
      };
    };

    connect();

    return () => {
      active = false;
      if (retryTimer.current !== null) {
        clearTimeout(retryTimer.current);
      }
      if (ws && ws.readyState === WebSocket.OPEN) {
        ws.close();
      }
    };
  }, [url]);

  return useMemo(
    () => ({
      socket,
      status,
      error,
      url,
    }),
    [error, socket, status, url],
  );
}
