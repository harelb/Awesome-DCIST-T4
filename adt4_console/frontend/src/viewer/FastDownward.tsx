import type {FastDownwardInfo} from "../lib/fastDownward";

/** Fast Downward's digest; the log tail and problem file fold out (failures). */
export function FastDownwardDetails({fd}: {fd: FastDownwardInfo | null | undefined}) {
  if (!fd || (!fd.digest && !fd.log?.length)) return null;
  return (
    <details className="small fd-details" data-testid="fast-downward">
      <summary className="muted">Fast Downward{fd.digest ? `: ${fd.digest}` : ""}</summary>
      {fd.log && fd.log.length > 0 && <pre className="mono small">{fd.log.join("\n")}</pre>}
      {fd.problem_file && <div className="muted mono small">problem: {fd.problem_file}</div>}
    </details>
  );
}
