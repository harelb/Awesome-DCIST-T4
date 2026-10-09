// Token handling: `?token=` is read once from the URL and kept in sessionStorage.
const KEY = "adt4_console_token";
let cached: string | null | undefined;

function safeGet(): string | null {
  try {
    return sessionStorage.getItem(KEY);
  } catch {
    return null;
  }
}

export function initToken(): string | null {
  const url = new URL(window.location.href);
  const t = url.searchParams.get("token");
  if (t) {
    try {
      sessionStorage.setItem(KEY, t);
    } catch {
      /* private mode: keep in memory */
    }
    cached = t;
    url.searchParams.delete("token");
    window.history.replaceState(window.history.state, "", url.toString());
    return t;
  }
  cached = safeGet();
  return cached;
}

export function getToken(): string | null {
  if (cached === undefined) cached = safeGet();
  return cached;
}

export function setToken(t: string | null) {
  cached = t;
  try {
    if (t) sessionStorage.setItem(KEY, t);
    else sessionStorage.removeItem(KEY);
  } catch {
    /* ignore */
  }
}

/** Append ?token= to a same-origin URL (images, websocket). */
export function withToken(url: string): string {
  const t = getToken();
  if (!t) return url;
  return url + (url.includes("?") ? "&" : "?") + "token=" + encodeURIComponent(t);
}
