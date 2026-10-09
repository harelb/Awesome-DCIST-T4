import {useState} from "react";
import {deleteJSON, errorText, postJSON, putJSON, useApi} from "../lib/api";
import {useFleet} from "../stores/fleet";
import type {ConsoleConfig, Topology} from "../types";
import {Badge, Card, Empty, ErrorBox, Field, KV, Loading, Modal, PageHeader, Spinner} from "../components/ui";

interface Edit {
  original: string | null;
  name: string;
  role: string;
  platform: string;
  desc: string;
  mit_wifi: string;
  silvus: string;
}

const IP_RE = /^(\d{1,3})\.(\d{1,3})\.(\d{1,3})\.(\d{1,3})$/;
export const validIp = (s: string) => !s || (IP_RE.test(s) && s.split(".").every(p => Number(p) <= 255));

export function ConfigPage() {
  const topo = useApi<Topology>("api/topology");
  const config = useApi<ConsoleConfig>("api/config");
  const [edit, setEdit] = useState<Edit | null>(null);
  const [del, setDel] = useState<string | null>(null);
  const [dirty, setDirty] = useState(false);
  const [busy, setBusy] = useState(false);
  const [msg, setMsg] = useState<{ok: boolean; text: string} | null>(null);

  const machines = Object.entries(topo.data?.machines ?? {}).sort(
    ([a, ma], [b, mb]) => ma.role.localeCompare(mb.role) || a.localeCompare(b),
  );
  const networks = [...new Set([...Object.keys(topo.data?.networks ?? {}), ...machines.flatMap(([, m]) => Object.keys(m.addresses ?? {}))])];

  const save = async () => {
    setBusy(true);
    setMsg(null);
    try {
      await postJSON("api/topology/save");
      setDirty(false);
      setMsg({ok: true, text: "topology.yaml saved"});
      void useFleet.getState().load();
    } catch (e) {
      setMsg({ok: false, text: errorText(e)});
    } finally {
      setBusy(false);
    }
  };

  const apply = async (e: Edit) => {
    const key = e.original ?? e.name;
    await putJSON(`api/topology/machines/${encodeURIComponent(key)}`, {
      name: e.name,
      role: e.role,
      platform: e.platform,
      desc: e.desc,
      mit_wifi: e.mit_wifi,
      silvus: e.silvus,
    });
    setDirty(true);
    setEdit(null);
    topo.reload();
  };

  const remove = async (name: string) => {
    try {
      await deleteJSON(`api/topology/machines/${encodeURIComponent(name)}`);
      setDirty(true);
      topo.reload();
    } catch (e) {
      setMsg({ok: false, text: errorText(e)});
    } finally {
      setDel(null);
    }
  };

  const c = config.data;
  return (
    <div className="page">
      <PageHeader
        title="Config"
        sub="Network topology (topology.yaml) and console settings."
        actions={
          <>
            {dirty && <Badge tone="warn">unsaved changes</Badge>}
            <button className="btn" onClick={() => setEdit({original: null, name: "", role: "robot", platform: "", desc: "", mit_wifi: "", silvus: ""})}>
              Add machine
            </button>
            <button className="btn btn-primary" onClick={save} disabled={busy || !dirty}>
              {busy && <Spinner />} Save
            </button>
          </>
        }
      />
      {msg && <div className={`banner ${msg.ok ? "banner-ok" : "banner-err"}`}>{msg.text}</div>}
      <Card title="Machines">
        {topo.loading && !topo.data && <Loading />}
        {topo.error && !topo.data && <ErrorBox error={topo.error} onRetry={topo.reload} />}
        {topo.data && machines.length === 0 && <Empty title="No machines in topology" />}
        {machines.length > 0 && (
          <div className="table-wrap">
            <table className="table">
              <thead>
                <tr>
                  <th>name</th>
                  <th>role</th>
                  <th>platform</th>
                  <th>description</th>
                  {networks.map(n => (
                    <th key={n} className="mono">
                      {n}
                      {c?.network === n && <Badge tone="info">active</Badge>}
                    </th>
                  ))}
                  <th />
                </tr>
              </thead>
              <tbody>
                {machines.map(([name, m]) => (
                  <tr key={name}>
                    <td className="strong">{name}</td>
                    <td>
                      <Badge tone={m.role === "robot" ? "info" : "off"}>{m.role.replace("_", " ")}</Badge>
                    </td>
                    <td className="mono small">
                      {m.platform_id ?? "-"}
                      {m.platform_type ? <span className="muted"> ({m.platform_type})</span> : null}
                    </td>
                    <td>{m.desc ?? ""}</td>
                    {networks.map(n => (
                      <td key={n} className="mono small">
                        {m.addresses?.[n] ?? <span className="muted">-</span>}
                      </td>
                    ))}
                    <td>
                      <div className="row gap-sm end">
                        <button
                          className="btn btn-sm"
                          onClick={() =>
                            setEdit({
                              original: name,
                              name,
                              role: m.role,
                              platform: m.platform_id ?? "",
                              desc: m.desc ?? "",
                              mit_wifi: m.addresses?.mit_wifi ?? "",
                              silvus: m.addresses?.silvus ?? "",
                            })
                          }
                        >
                          Edit
                        </button>
                        <button className="btn btn-sm btn-danger-outline" onClick={() => setDel(name)}>
                          Remove
                        </button>
                      </div>
                    </td>
                  </tr>
                ))}
              </tbody>
            </table>
          </div>
        )}
      </Card>

      <Card title="Console">
        {config.error && !c && <ErrorBox error={config.error} compact onRetry={config.reload} />}
        {c && (
          <div className="kv-grid">
            <KV k="active network" mono>
              {c.network}
            </KV>
            <KV k="local machine" mono>
              {c.local_machine}
            </KV>
            <KV k="output root" mono>
              {c.output_root}
            </KV>
            <KV k="robots (planning)" mono>
              {(c.robots ?? []).join(", ") || "-"}
            </KV>
            <KV k="neo4j targets" mono>
              {(c.neo4j_targets ?? []).length
                ? c.neo4j_targets.map(t => (
                    <div key={t.name}>
                      {t.name}
                      {t.prior ? " (prior)" : ""} {t.uri}
                    </div>
                  ))
                : "-"}
            </KV>
            <KV k="file maps" mono>
              {(c.maps ?? []).map(m => m.name).join(", ") || "-"}
            </KV>
          </div>
        )}
      </Card>

      {edit && <EditDialog edit={edit} onClose={() => setEdit(null)} onApply={apply} />}
      {del && (
        <Modal
          title={`Remove ${del}?`}
          tone="warn"
          onClose={() => setDel(null)}
          footer={
            <>
              <button className="btn" onClick={() => setDel(null)}>
                Cancel
              </button>
              <button className="btn btn-danger" onClick={() => remove(del)}>
                Remove
              </button>
            </>
          }
        >
          <p>
            Removes <span className="mono">{del}</span> from the in-memory topology. Press Save to write topology.yaml.
          </p>
        </Modal>
      )}
    </div>
  );
}

function EditDialog({edit, onClose, onApply}: {edit: Edit; onClose: () => void; onApply: (e: Edit) => Promise<void>}) {
  const [e, setE] = useState(edit);
  const [err, setErr] = useState<string | null>(null);
  const [busy, setBusy] = useState(false);
  const set = (k: keyof Edit) => (ev: {target: {value: string}}) => setE({...e, [k]: ev.target.value});
  const problems = [
    !e.name.trim() && "name is required",
    !/^[\w-]*$/.test(e.name) && "name: letters, digits, _ and - only",
    !validIp(e.mit_wifi) && "mit_wifi is not an IPv4 address",
    !validIp(e.silvus) && "silvus is not an IPv4 address",
  ].filter(Boolean) as string[];
  return (
    <Modal
      title={edit.original ? `Edit ${edit.original}` : "Add machine"}
      onClose={onClose}
      footer={
        <>
          <span className="tone-warn small">{problems[0]}</span>
          <button className="btn" onClick={onClose}>
            Cancel
          </button>
          <button
            className="btn btn-primary"
            disabled={busy || problems.length > 0}
            onClick={async () => {
              setBusy(true);
              try {
                await onApply(e);
              } catch (x) {
                setErr(errorText(x));
              } finally {
                setBusy(false);
              }
            }}
          >
            Apply
          </button>
        </>
      }
    >
      <div className="form-grid">
        <Field label="Name">
          <input className="input mono" value={e.name} onChange={set("name")} autoFocus />
        </Field>
        <Field label="Role">
          <select className="input" value={e.role} onChange={set("role")}>
            <option value="robot">robot</option>
            <option value="base_station">base station</option>
          </select>
        </Field>
        <Field label="Platform id">
          <input className="input mono" value={e.platform} onChange={set("platform")} placeholder="smaug, topaz, samoyed" />
        </Field>
        <Field label="Description">
          <input className="input" value={e.desc} onChange={set("desc")} />
        </Field>
        <Field label="mit_wifi address">
          <input className="input mono" value={e.mit_wifi} onChange={set("mit_wifi")} placeholder="10.29.x.x" />
        </Field>
        <Field label="silvus address">
          <input className="input mono" value={e.silvus} onChange={set("silvus")} placeholder="192.168.100.x" />
        </Field>
      </div>
      <p className="muted small">Addresses on other networks and extra keys of the entry are preserved.</p>
      {err && <ErrorBox error={err} compact />}
    </Modal>
  );
}
