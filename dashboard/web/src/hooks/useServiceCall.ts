import { useCallback, useRef } from "react";

type ServiceCallResult<T> = {
  values?: T;
  result?: boolean;
};

type RosBridgeServiceResponse<T> = {
  op: string;
  id?: string;
  service?: string;
  values?: T;
  result?: boolean;
};

let callCounter = 0;

/**
 * Calls a ROS2 service via rosbridge WebSocket `call_service` operation.
 * Returns the service response values or throws on failure.
 */
export function callService<TReq, TRes>(
  socket: WebSocket,
  service: string,
  type: string,
  request: TReq,
  timeoutMs = 10000,
): Promise<ServiceCallResult<TRes>> {
  return new Promise((resolve, reject) => {
    const id = `call_service:${service}:${++callCounter}`;

    const timer = setTimeout(() => {
      socket.removeEventListener("message", onMessage);
      reject(new Error(`Service call to ${service} timed out after ${timeoutMs}ms`));
    }, timeoutMs);

    const onMessage = (event: MessageEvent) => {
      try {
        const parsed = JSON.parse(event.data) as RosBridgeServiceResponse<TRes>;
        if (parsed.op !== "service_response" || parsed.id !== id) {
          return;
        }
        clearTimeout(timer);
        socket.removeEventListener("message", onMessage);
        if (parsed.result === false) {
          reject(new Error(`Service ${service} returned failure`));
        } else {
          resolve({ values: parsed.values, result: parsed.result });
        }
      } catch {
        // Ignore unrelated messages
      }
    };

    socket.addEventListener("message", onMessage);
    socket.send(
      JSON.stringify({
        op: "call_service",
        id,
        service,
        type,
        args: request,
      }),
    );
  });
}

/** Parameter value types matching rcl_interfaces/msg/ParameterValue */
export const PARAM_TYPE = {
  NOT_SET: 0,
  BOOL: 1,
  INTEGER: 2,
  DOUBLE: 3,
  STRING: 4,
  BYTE_ARRAY: 5,
  BOOL_ARRAY: 6,
  INTEGER_ARRAY: 7,
  DOUBLE_ARRAY: 8,
  STRING_ARRAY: 9,
} as const;

export type ParameterValue = {
  type: number;
  bool_value?: boolean;
  integer_value?: number;
  double_value?: number;
  string_value?: string;
  double_array_value?: number[];
};

export type SetParametersRequest = {
  parameters: Array<{
    name: string;
    value: ParameterValue;
  }>;
};

export type SetParametersResponse = {
  results: Array<{
    successful: boolean;
    reason: string;
  }>;
};

export type GetParametersRequest = {
  names: string[];
};

export type GetParametersResponse = {
  values: ParameterValue[];
};

/**
 * Hook that returns a stable setParams function for setting ROS2 parameters.
 */
/** Tagged value to force a specific ROS parameter type. */
export type TypedParamValue =
  | { _type: "double"; value: number }
  | { _type: "int"; value: number };

/** Force a number to be sent as DOUBLE (avoids Number.isInteger misdetection). */
export function asDouble(v: number): TypedParamValue {
  return { _type: "double", value: v };
}

export function useSetParams(socket: WebSocket | null, node: string, timeoutMs = 10000) {
  const socketRef = useRef(socket);
  socketRef.current = socket;

  return useCallback(
    async (params: Record<string, boolean | number | string | number[] | TypedParamValue>) => {
      const ws = socketRef.current;
      if (!ws || ws.readyState !== WebSocket.OPEN) {
        throw new Error("WebSocket not connected");
      }

      const parameters = Object.entries(params).map(([name, value]) => {
        const pv: ParameterValue = { type: PARAM_TYPE.NOT_SET };
        if (typeof value === "object" && value !== null && "_type" in value) {
          // Explicitly typed value
          if (value._type === "double") {
            pv.type = PARAM_TYPE.DOUBLE;
            pv.double_value = value.value;
          } else {
            pv.type = PARAM_TYPE.INTEGER;
            pv.integer_value = value.value;
          }
        } else if (typeof value === "boolean") {
          pv.type = PARAM_TYPE.BOOL;
          pv.bool_value = value;
        } else if (typeof value === "number") {
          if (Number.isInteger(value)) {
            pv.type = PARAM_TYPE.INTEGER;
            pv.integer_value = value;
          } else {
            pv.type = PARAM_TYPE.DOUBLE;
            pv.double_value = value;
          }
        } else if (typeof value === "string") {
          pv.type = PARAM_TYPE.STRING;
          pv.string_value = value;
        } else if (Array.isArray(value)) {
          pv.type = PARAM_TYPE.DOUBLE_ARRAY;
          pv.double_array_value = value;
        }
        return { name, value: pv };
      });

      const resp = await callService<SetParametersRequest, SetParametersResponse>(
        ws,
        `/${node}/set_parameters`,
        "rcl_interfaces/srv/SetParameters",
        { parameters },
        timeoutMs,
      );

      return resp.values?.results ?? [];
    },
    [node, timeoutMs],
  );
}

/**
 * Hook that returns a stable getParams function for getting ROS2 parameters.
 */
export function useGetParams(socket: WebSocket | null, node: string, timeoutMs = 10000) {
  const socketRef = useRef(socket);
  socketRef.current = socket;

  return useCallback(
    async (names: string[]) => {
      const ws = socketRef.current;
      if (!ws || ws.readyState !== WebSocket.OPEN) {
        throw new Error("WebSocket not connected");
      }

      const resp = await callService<GetParametersRequest, GetParametersResponse>(
        ws,
        `/${node}/get_parameters`,
        "rcl_interfaces/srv/GetParameters",
        { names },
        timeoutMs,
      );

      return resp.values?.values ?? [];
    },
    [node, timeoutMs],
  );
}
