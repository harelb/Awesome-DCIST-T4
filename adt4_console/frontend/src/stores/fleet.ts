import {create} from "zustand";
import type {FleetMachines, FleetNodes, FleetStatus, Health} from "../types";
import {ws} from "../ws";
import {ApiError, getJSON} from "../lib/api";

interface FleetStore {
  machines: FleetMachines | null;
  status: FleetStatus | null;
  nodes: FleetNodes | null;
  nodesError: ApiError | null;
  health: Health | null;
  loaded: boolean;
  error: ApiError | null;
  load: () => Promise<void>;
}

export const useFleet = create<FleetStore>(set => ({
  machines: null,
  status: null,
  nodes: null,
  nodesError: null,
  health: null,
  loaded: false,
  error: null,
  load: async () => {
    const [m, s, n, h] = await Promise.allSettled([
      getJSON<FleetMachines>("api/fleet"),
      getJSON<FleetStatus>("api/fleet/status"),
      getJSON<FleetNodes>("api/fleet/nodes"),
      getJSON<Health>("api/health"),
    ]);
    set(cur => ({
      machines: m.status === "fulfilled" ? m.value : cur.machines,
      status: s.status === "fulfilled" ? s.value : cur.status,
      nodes: n.status === "fulfilled" ? n.value : cur.nodes,
      nodesError: n.status === "rejected" ? (n.reason as ApiError) : null,
      health: h.status === "fulfilled" ? h.value : cur.health,
      loaded: true,
      error: m.status === "rejected" ? (m.reason as ApiError) : null,
    }));
  },
}));

export function bindFleet() {
  ws.on("fleet/machines", d => useFleet.setState({machines: d as FleetMachines, error: null}));
  ws.on("fleet/status", d => useFleet.setState({status: d as FleetStatus}));
  ws.on("fleet/nodes", d => useFleet.setState({nodes: d as FleetNodes, nodesError: null}));
  ws.onReconnect(() => void useFleet.getState().load());
}
