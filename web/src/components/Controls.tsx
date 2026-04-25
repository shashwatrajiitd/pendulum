import type { LinkConfig } from "../engine/wasm-bridge";

interface Props {
  N: number;
  links: LinkConfig[];
  g: number;
  theta0: number[];
  thetaDot0: number[];
  playing: boolean;
  onNChange: (n: number) => void;
  onLinksChange: (links: LinkConfig[]) => void;
  onGChange: (g: number) => void;
  onTheta0Change: (t: number[]) => void;
  onThetaDot0Change: (t: number[]) => void;
  onPlayPause: () => void;
  onReset: () => void;
  onStep: () => void;
}

function LinkEditor({
  idx,
  link,
  theta,
  thetaDot,
  onChange,
  onThetaChange,
  onThetaDotChange,
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
    <div style={{ marginBottom: 12, padding: 8, background: "#f5f5f5", borderRadius: 6 }}>
      <strong>Link {idx + 1}</strong>
      <div style={{ display: "grid", gridTemplateColumns: "1fr 1fr", gap: 4, marginTop: 4 }}>
        <label>
          L
          <input type="number" step="0.1" min="0.1" value={link.L}
            onChange={(e) => onChange({ ...link, L: +e.target.value })}
            style={{ width: 60, marginLeft: 4 }} />
        </label>
        <label>
          m_bob
          <input type="number" step="0.1" min="0.01" value={link.m_bob}
            onChange={(e) => onChange({ ...link, m_bob: +e.target.value })}
            style={{ width: 60, marginLeft: 4 }} />
        </label>
        <label>
          m_rod
          <input type="number" step="0.1" min="0" value={link.m_rod}
            onChange={(e) => onChange({ ...link, m_rod: +e.target.value })}
            style={{ width: 60, marginLeft: 4 }} />
        </label>
        <label>
          r_bob
          <input type="number" step="0.01" min="0" value={link.r_bob}
            onChange={(e) => onChange({ ...link, r_bob: +e.target.value })}
            style={{ width: 60, marginLeft: 4 }} />
        </label>
        <label>
          θ₀
          <input type="number" step="0.1" value={theta}
            onChange={(e) => onThetaChange(+e.target.value)}
            style={{ width: 60, marginLeft: 4 }} />
        </label>
        <label>
          θ̇₀
          <input type="number" step="0.1" value={thetaDot}
            onChange={(e) => onThetaDotChange(+e.target.value)}
            style={{ width: 60, marginLeft: 4 }} />
        </label>
      </div>
    </div>
  );
}

export function Controls(props: Props) {
  return (
    <div style={{ padding: 16, width: 280 }}>
      <h3 style={{ margin: "0 0 12px" }}>Parameters</h3>

      <label style={{ display: "block", marginBottom: 12 }}>
        Pendulum count (N):{" "}
        <select value={props.N} onChange={(e) => props.onNChange(+e.target.value)}>
          <option value={1}>1</option>
          <option value={2}>2</option>
          <option value={3}>3</option>
        </select>
      </label>

      <label style={{ display: "block", marginBottom: 12 }}>
        g:{" "}
        <input type="number" step="0.1" value={props.g}
          onChange={(e) => props.onGChange(+e.target.value)}
          style={{ width: 80 }} />
      </label>

      {props.links.slice(0, props.N).map((link, i) => (
        <LinkEditor
          key={i}
          idx={i}
          link={link}
          theta={props.theta0[i]}
          thetaDot={props.thetaDot0[i]}
          onChange={(l) => {
            const newLinks = [...props.links];
            newLinks[i] = l;
            props.onLinksChange(newLinks);
          }}
          onThetaChange={(v) => {
            const t = [...props.theta0];
            t[i] = v;
            props.onTheta0Change(t);
          }}
          onThetaDotChange={(v) => {
            const t = [...props.thetaDot0];
            t[i] = v;
            props.onThetaDot0Change(t);
          }}
        />
      ))}

      <div style={{ display: "flex", gap: 8, marginTop: 16 }}>
        <button onClick={props.onPlayPause}
          style={{ flex: 1, padding: "8px 0", fontSize: 16, cursor: "pointer" }}>
          {props.playing ? "⏸ Pause" : "▶ Play"}
        </button>
        <button onClick={props.onStep}
          style={{ padding: "8px 12px", cursor: "pointer" }}>
          Step
        </button>
        <button onClick={props.onReset}
          style={{ padding: "8px 12px", cursor: "pointer" }}>
          Reset
        </button>
      </div>
    </div>
  );
}
