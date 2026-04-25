import type { PendulumState } from "../engine/wasm-bridge";

interface Props {
  state: PendulumState;
  fps: number;
}

export function InfoPanel({ state, fps }: Props) {
  return (
    <div
      style={{
        padding: 12,
        background: "#f9f9f9",
        borderRadius: 6,
        fontFamily: "monospace",
        fontSize: 13,
        lineHeight: 1.6,
      }}
    >
      <div>
        <strong>t</strong> = {state.t.toFixed(3)} s
      </div>
      <div>
        <strong>E</strong> = {state.energy.toFixed(8)} J
      </div>
      <div>
        <strong>|ΔE/E₀|</strong> = {state.energy_drift.toExponential(3)}
      </div>
      {state.theta.map((th, i) => (
        <div key={i}>
          <strong>θ{i + 1}</strong> = {th.toFixed(4)} rad &nbsp;
          <strong>θ̇{i + 1}</strong> = {state.theta_dot[i].toFixed(4)} rad/s
        </div>
      ))}
      <div>
        <strong>FPS</strong> = {fps}
      </div>
    </div>
  );
}
