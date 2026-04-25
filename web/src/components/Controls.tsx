import type { LinkConfig } from "../engine/wasm-bridge";

export interface PendulumSystem {
  N: number;
  links: LinkConfig[];
  theta0: number[];
  thetaDot0: number[];
}

interface Props {
  systems: PendulumSystem[];
  g: number;
  playing: boolean;
  speed: number;
  selectedSystem: number;
  onSystemsChange: (systems: PendulumSystem[]) => void;
  onGChange: (g: number) => void;
  onPlayPause: () => void;
  onReset: () => void;
  onStep: () => void;
  onSpeedChange: (s: number) => void;
  onSelectedSystemChange: (i: number) => void;
}

const SYSTEM_COLORS = ["#e74c3c", "#3498db", "#2ecc71"];

function LinkEditor({
  idx, link, theta, thetaDot, onChange, onThetaChange, onThetaDotChange,
}: {
  idx: number;
  link: LinkConfig;
  theta: number;
  thetaDot: number;
  onChange: (l: LinkConfig) => void;
  onThetaChange: (v: number) => void;
  onThetaDotChange: (v: number) => void;
}) {
  return (
    <div style={{ marginBottom: 8, padding: 6, background: "#f0f0f0", borderRadius: 4 }}>
      <strong style={{ fontSize: 12 }}>Link {idx + 1}</strong>
      <div style={{ display: "grid", gridTemplateColumns: "1fr 1fr", gap: 3, marginTop: 3 }}>
        <label style={{ fontSize: 12 }}>
          L
          <input type="number" step="0.1" min="0.1" value={link.L}
            onChange={(e) => onChange({ ...link, L: +e.target.value })}
            style={{ width: 50, marginLeft: 4 }} />
        </label>
        <label style={{ fontSize: 12 }}>
          m_bob
          <input type="number" step="0.1" min="0.01" value={link.m_bob}
            onChange={(e) => onChange({ ...link, m_bob: +e.target.value })}
            style={{ width: 50, marginLeft: 4 }} />
        </label>
        <label style={{ fontSize: 12 }}>
          m_rod
          <input type="number" step="0.1" min="0" value={link.m_rod}
            onChange={(e) => onChange({ ...link, m_rod: +e.target.value })}
            style={{ width: 50, marginLeft: 4 }} />
        </label>
        <label style={{ fontSize: 12 }}>
          r_bob
          <input type="number" step="0.01" min="0" value={link.r_bob}
            onChange={(e) => onChange({ ...link, r_bob: +e.target.value })}
            style={{ width: 50, marginLeft: 4 }} />
        </label>
        <label style={{ fontSize: 12 }}>
          &theta;&#8320;
          <input type="number" step="0.1" value={theta}
            onChange={(e) => onThetaChange(+e.target.value)}
            style={{ width: 50, marginLeft: 4 }} />
        </label>
        <label style={{ fontSize: 12 }}>
          &theta;&#775;&#8320;
          <input type="number" step="0.1" value={thetaDot}
            onChange={(e) => onThetaDotChange(+e.target.value)}
            style={{ width: 50, marginLeft: 4 }} />
        </label>
      </div>
    </div>
  );
}

function SystemEditor({
  sysIdx, system, color, selected, onSelect, onChange, onRemove,
}: {
  sysIdx: number;
  system: PendulumSystem;
  color: string;
  selected: boolean;
  onSelect: () => void;
  onChange: (s: PendulumSystem) => void;
  onRemove: (() => void) | null;
}) {
  return (
    <details open={selected} style={{ marginBottom: 8 }}>
      <summary
        onClick={(e) => { e.preventDefault(); onSelect(); }}
        style={{
          cursor: "pointer", padding: "6px 8px", borderRadius: 6,
          background: selected ? color + "22" : "#f9f9f9",
          borderLeft: `4px solid ${color}`,
          display: "flex", justifyContent: "space-between", alignItems: "center",
          fontSize: 13, fontWeight: 600,
        }}
      >
        <span>System {sysIdx + 1}</span>
        <span style={{ display: "flex", gap: 8, alignItems: "center" }}>
          <span style={{ fontSize: 11, fontWeight: 400, color: "#888" }}>
            N={system.N}
          </span>
          {onRemove && (
            <button onClick={(e) => { e.stopPropagation(); onRemove(); }}
              style={{ background: "none", border: "none", cursor: "pointer", color: "#c00", fontSize: 14, padding: 0 }}>
              &times;
            </button>
          )}
        </span>
      </summary>
      {selected && (
        <div style={{ padding: "8px 0 0 0" }}>
          <label style={{ display: "block", marginBottom: 8, fontSize: 12 }}>
            Chain length (N):{" "}
            <select value={system.N} onChange={(e) => onChange({ ...system, N: +e.target.value })}>
              <option value={1}>1</option>
              <option value={2}>2</option>
              <option value={3}>3</option>
            </select>
          </label>
          {system.links.slice(0, system.N).map((link, i) => (
            <LinkEditor
              key={i} idx={i} link={link}
              theta={system.theta0[i]} thetaDot={system.thetaDot0[i]}
              onChange={(l) => {
                const newLinks = [...system.links];
                newLinks[i] = l;
                onChange({ ...system, links: newLinks });
              }}
              onThetaChange={(v) => {
                const t = [...system.theta0];
                t[i] = v;
                onChange({ ...system, theta0: t });
              }}
              onThetaDotChange={(v) => {
                const t = [...system.thetaDot0];
                t[i] = v;
                onChange({ ...system, thetaDot0: t });
              }}
            />
          ))}
        </div>
      )}
    </details>
  );
}

export function Controls(props: Props) {
  const updateSystem = (idx: number, sys: PendulumSystem) => {
    const next = [...props.systems];
    next[idx] = sys;
    props.onSystemsChange(next);
  };

  const addSystem = () => {
    if (props.systems.length >= 3) return;
    const defaults: PendulumSystem = {
      N: 1,
      links: [
        { L: 1.0, m_rod: 0, m_bob: 1.0, r_bob: 0.05 },
        { L: 1.0, m_rod: 0, m_bob: 1.0, r_bob: 0.05 },
        { L: 1.0, m_rod: 0, m_bob: 1.0, r_bob: 0.05 },
      ],
      theta0: [1.5, 1.0, 0.5],
      thetaDot0: [0, 0, 0],
    };
    props.onSystemsChange([...props.systems, defaults]);
    props.onSelectedSystemChange(props.systems.length);
  };

  const removeSystem = (idx: number) => {
    const next = props.systems.filter((_, i) => i !== idx);
    props.onSystemsChange(next);
    if (props.selectedSystem >= next.length)
      props.onSelectedSystemChange(Math.max(0, next.length - 1));
  };

  return (
    <div style={{ padding: 12, width: 280, overflowY: "auto", borderRight: "1px solid #eee" }}>
      <h3 style={{ margin: "0 0 8px", fontSize: 15 }}>Parameters</h3>

      <label style={{ display: "block", marginBottom: 10, fontSize: 13 }}>
        g:{" "}
        <input type="number" step="0.1" value={props.g}
          onChange={(e) => props.onGChange(+e.target.value)}
          style={{ width: 80 }} />
      </label>

      {props.systems.map((sys, i) => (
        <SystemEditor
          key={i} sysIdx={i} system={sys}
          color={SYSTEM_COLORS[i]}
          selected={i === props.selectedSystem}
          onSelect={() => props.onSelectedSystemChange(i)}
          onChange={(s) => updateSystem(i, s)}
          onRemove={props.systems.length > 1 ? () => removeSystem(i) : null}
        />
      ))}

      {props.systems.length < 3 && (
        <button onClick={addSystem}
          style={{ width: "100%", padding: "6px 0", marginBottom: 8, cursor: "pointer", fontSize: 13, border: "1px dashed #ccc", background: "none", borderRadius: 6 }}>
          + Add System
        </button>
      )}

      <label style={{ display: "block", marginTop: 8, fontSize: 13 }}>
        Speed: {props.speed >= 1 ? props.speed + "x" : "1/" + (1 / props.speed) + "x"}
        <input type="range" min="-3" max="3" step="1" value={Math.round(Math.log2(props.speed))}
          onChange={(e) => props.onSpeedChange(Math.pow(2, +e.target.value))}
          style={{ width: "100%", marginTop: 4 }} />
        <div style={{ display: "flex", justifyContent: "space-between", fontSize: 11, color: "#999" }}>
          <span>1/8x</span><span>1x</span><span>8x</span>
        </div>
      </label>

      <div style={{ display: "flex", gap: 8, marginTop: 8 }}>
        <button onClick={props.onPlayPause}
          style={{ flex: 1, padding: "8px 0", fontSize: 15, cursor: "pointer" }}>
          {props.playing ? "⏸ Pause" : "▶ Play"}
        </button>
        <button onClick={props.onStep} style={{ padding: "8px 12px", cursor: "pointer" }}>Step</button>
        <button onClick={props.onReset} style={{ padding: "8px 12px", cursor: "pointer" }}>Reset</button>
      </div>
    </div>
  );
}
