// The chat conversation, outside the ChatPanel so it survives switching pages
// (the panel unmounts) and a reload or another browser tab (localStorage).
// A turn still streaming when the panel unmounts keeps writing here.
import {create} from "zustand";
import type {ChatItem} from "../viewer/ChatPanel";
import type {TranscriptItem} from "../lib/chatTranscript";

export const CHAT_STORAGE_KEY = "adt4.chat.v1";
const MAX_ITEMS = 300;

interface ChatStore {
  items: ChatItem[];
  /** what the model sees next turn (server-built transcript) */
  transcript: TranscriptItem[];
  busy: boolean;
  /** index of the current turn's first item (scroll anchor) */
  turnStart: number;
  /** text to put in the message box (e.g. "Re-plan in chat"); the panel takes it */
  prefill: string | null;
}

/** What is kept on disk: no busy flag, and no embedded candidate images (data
 *  URIs can be hundreds of kB; a candidate's pending id expires anyway). */
export function storable(items: ChatItem[]): ChatItem[] {
  return items.slice(-MAX_ITEMS).map(it => (it.type === "object_candidate" && it.image_uri ? {...it, image_uri: null} : it));
}

function load(): Pick<ChatStore, "items" | "transcript"> {
  try {
    const raw = window.localStorage.getItem(CHAT_STORAGE_KEY);
    const v = raw ? JSON.parse(raw) : null;
    if (v && Array.isArray(v.items) && Array.isArray(v.transcript)) return {items: v.items, transcript: v.transcript};
  } catch {
    /* private window / bad data */
  }
  return {items: [], transcript: []};
}

export const useChat = create<ChatStore>(() => ({...load(), busy: false, turnStart: 0, prefill: null}));

/** Put `text` in the chat's message box (the mission planner shows the chat pane). */
export function prefillChat(text: string): void {
  useChat.setState({prefill: text});
}

let saveTimer: ReturnType<typeof setTimeout> | null = null;
useChat.subscribe((s, prev) => {
  if (s.items === prev.items && s.transcript === prev.transcript) return;
  if (saveTimer) clearTimeout(saveTimer);
  saveTimer = setTimeout(() => {
    try {
      window.localStorage.setItem(CHAT_STORAGE_KEY, JSON.stringify({items: storable(s.items), transcript: s.transcript}));
    } catch {
      /* quota / private window: the in-memory chat still works */
    }
  }, 300);
});

/** The in-flight request, so "New chat" can stop it from any mount. */
export const chatAbort: {current: AbortController | null} = {current: null};

export function clearChat(): void {
  chatAbort.current?.abort();
  chatAbort.current = null;
  useChat.setState({items: [], transcript: [], busy: false, turnStart: 0});
}
