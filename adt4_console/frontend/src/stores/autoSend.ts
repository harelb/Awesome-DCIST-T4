// Chat "Auto-send to online robots": off by default, remembered per viewer.
import {create} from "zustand";

export const AUTO_SEND_KEY = "adt4.chat.autoSend.v1";

function load(): boolean {
  try {
    return window.localStorage.getItem(AUTO_SEND_KEY) === "1";
  } catch {
    return false;
  }
}

export const useAutoSend = create<{on: boolean}>(() => ({on: load()}));

export function setAutoSend(on: boolean): void {
  useAutoSend.setState({on});
  try {
    window.localStorage.setItem(AUTO_SEND_KEY, on ? "1" : "0");
  } catch {
    /* private window: the toggle still works for this tab */
  }
}
