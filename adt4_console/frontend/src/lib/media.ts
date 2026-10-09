// Media-query hooks for the phone layout. PHONE_MQ must match the
// `max-width: 640px` breakpoint in styles.css.
import {useSyncExternalStore} from "react";

export const PHONE_MQ = "(max-width: 640px)";

const subscribers = new Map<string, (cb: () => void) => () => void>();

/** One stable subscribe function per query, so useSyncExternalStore does not resubscribe each render. */
function subscribe(query: string) {
  let s = subscribers.get(query);
  if (!s) {
    s = (cb: () => void) => {
      const mq = window.matchMedia?.(query);
      mq?.addEventListener?.("change", cb);
      return () => mq?.removeEventListener?.("change", cb);
    };
    subscribers.set(query, s);
  }
  return s;
}

export function useMedia(query: string): boolean {
  return useSyncExternalStore(
    subscribe(query),
    () => window.matchMedia?.(query).matches ?? false,
    () => false,
  );
}

export const useIsPhone = () => useMedia(PHONE_MQ);
