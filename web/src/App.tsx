import { useState, useRef, useCallback, useEffect } from "react";
import { PendulumEngine, type LinkConfig, type PendulumState } from "./engine/wasm-bridge";
import { Controls, type PendulumSystem } from "./components/Controls";

const DEFAULT_SYSTEMS: PendulumSystem[] = [
  {
    N: 2,
    links: [
      { L: 1.0, m_rod: 0, m_bob: 1.0, r_bob: 0.05 },
      { L: 1.0, m_rod: 0, m_bob: 1.0, r_bob: 0.05 },
      { L: 1.0, m_rod: 0, m_bob: 1.0, r_bob: 0.05 },
    ],
    theta0: [2.0, 2.0, 0.5],
    thetaDot0: [0, 0, 0],
  },
];

const EMPTY_STATE: PendulumState = {
  theta: [0], theta_dot: [0], t: 0, energy: 0, energy_drift: 0,
  positions: [{ x: 0, y: 0 }],
};

const TRAIL_MAX = 800;
const ENERGY_MAX = 600;
const PHASE_MAX = 600;
const MOBILE_BREAKPOINT = 768;

function useTheme() {
  const [theme, setTheme] = useState<"light" | "dark">(() => {
    if (typeof window !== "undefined" && window.matchMedia("(prefers-color-scheme: dark)").matches)
      return "dark";
    return "light";
  });
  useEffect(() => {
    document.documentElement.setAttribute("data-theme", theme);
  }, [theme]);
  const toggle = useCallback(() => setTheme(t => t === "light" ? "dark" : "light"), []);
  return { theme, toggle };
}

const SYSTEM_COLORS = ["#e74c3c", "#3498db", "#2ecc71"];
const SYSTEM_TRAIL_COLORS = [
  "rgba(231, 76, 60, 0.3)",
  "rgba(52, 152, 219, 0.3)",
  "rgba(46, 204, 113, 0.3)",
];
const SYSTEM_BOB_COLORS = [
  ["#e74c3c", "#c0392b", "#a93226"],
  ["#3498db", "#2980b9", "#2471a3"],
  ["#2ecc71", "#27ae60", "#229954"],
];

interface PerSystemData {
  trail: { x: number; y: number }[];
  energy: { t: number; dE: number }[];
  phase: { theta: number; thetaDot: number }[][];
  e0: number;
}

function useWindowWidth() {
  const [width, setWidth] = useState(window.innerWidth);
  useEffect(() => {
    const onResize = () => setWidth(window.innerWidth);
    window.addEventListener("resize", onResize);
    return () => window.removeEventListener("resize", onResize);
  }, []);
  return width;
}

function drawPendulumCanvas(
  canvas: HTMLCanvasElement,
  states: PendulumState[],
  allLinks: LinkConfig[][],
  trails: { x: number; y: number }[][],
  dark = false,
) {
  const ctx = canvas.getContext("2d");
  if (!ctx) return;
  const w = canvas.width, h = canvas.height;

  let maxTotalL = 0;
  for (const links of allLinks) {
    const total = links.reduce((s, l) => s + l.L, 0);
    if (total > maxTotalL) maxTotalL = total;
  }
  if (maxTotalL === 0) maxTotalL = 1;
  const scale = (Math.min(w, h) * 0.35) / maxTotalL;
  const ox = w / 2, oy = h * 0.35;

  ctx.clearRect(0, 0, w, h);

  for (let si = 0; si < trails.length; si++) {
    const trail = trails[si];
    if (trail.length > 1) {
      ctx.beginPath();
      ctx.strokeStyle = SYSTEM_TRAIL_COLORS[si];
      ctx.lineWidth = 1.5;
      for (let i = 0; i < trail.length; i++) {
        const sx = ox + trail[i].x * scale;
        const sy = oy - trail[i].y * scale;
        if (i === 0) ctx.moveTo(sx, sy); else ctx.lineTo(sx, sy);
      }
      ctx.stroke();
    }
  }

  ctx.fillStyle = dark ? "#aaa" : "#333";
  ctx.beginPath();
  ctx.arc(ox, oy, 4, 0, 2 * Math.PI);
  ctx.fill();

  for (let si = 0; si < states.length; si++) {
    const state = states[si];
    const links = allLinks[si];
    const colors = SYSTEM_BOB_COLORS[si];
    let prevX = ox, prevY = oy;
    for (let i = 0; i < state.positions.length; i++) {
      const bx = ox + state.positions[i].x * scale;
      const by = oy - state.positions[i].y * scale;
      ctx.beginPath();
      ctx.strokeStyle = SYSTEM_COLORS[si] + (dark ? "aa" : "88");
      ctx.lineWidth = 2.5;
      ctx.moveTo(prevX, prevY);
      ctx.lineTo(bx, by);
      ctx.stroke();
      const bobR = Math.max(links[i].r_bob * scale, 6);
      ctx.beginPath();
      ctx.fillStyle = colors[i];
      ctx.arc(bx, by, bobR, 0, 2 * Math.PI);
      ctx.fill();
      ctx.strokeStyle = dark ? "#555" : "#333";
      ctx.lineWidth = 1;
      ctx.stroke();
      prevX = bx; prevY = by;
    }
  }
}

function drawEnergyCanvas(
  canvas: HTMLCanvasElement,
  allData: { t: number; dE: number }[][],
  dark = false,
) {
  const ctx = canvas.getContext("2d");
  if (!ctx) return;
  const w = canvas.width, h = canvas.height;
  ctx.clearRect(0, 0, w, h);

  const margin = { top: 20, right: 10, bottom: 30, left: 60 };
  const cw = w - margin.left - margin.right;
  const ch = h - margin.top - margin.bottom;

  let tMax = 1, absMax = 1e-15;
  for (const data of allData) {
    if (data.length < 2) continue;
    const tl = data[data.length - 1].t;
    if (tl > tMax) tMax = tl;
    for (const d of data) {
      const a = Math.abs(d.dE);
      if (a > absMax) absMax = a;
    }
  }
  absMax *= 1.2;

  ctx.save();
  ctx.translate(margin.left, margin.top);
  ctx.strokeStyle = dark ? "#3a3a5a" : "#ccc";
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.moveTo(0, 0); ctx.lineTo(0, ch); ctx.lineTo(cw, ch);
  ctx.stroke();

  const y0 = ch / 2;
  ctx.strokeStyle = dark ? "#2a2a4a" : "#ddd";
  ctx.setLineDash([4, 4]);
  ctx.beginPath();
  ctx.moveTo(0, y0); ctx.lineTo(cw, y0);
  ctx.stroke();
  ctx.setLineDash([]);

  for (let si = 0; si < allData.length; si++) {
    const data = allData[si];
    if (data.length < 2) continue;
    ctx.beginPath();
    ctx.strokeStyle = SYSTEM_COLORS[si];
    ctx.lineWidth = 1.5;
    for (let i = 0; i < data.length; i++) {
      const x = (data[i].t / tMax) * cw;
      const y = y0 - (data[i].dE / absMax) * (ch / 2);
      if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
    }
    ctx.stroke();
  }
  ctx.restore();

  ctx.fillStyle = dark ? "#8888aa" : "#666";
  ctx.font = "11px monospace";
  ctx.fillText("E(t) - E₀", margin.left + 4, margin.top - 6);
  ctx.fillText(`t = ${tMax.toFixed(1)}s`, w - margin.right - 60, h - 6);
  ctx.fillText(`±${absMax.toExponential(1)}`, 2, margin.top + ch / 2 - 4);
}

function drawPhaseCanvas(
  canvas: HTMLCanvasElement,
  data: { theta: number; thetaDot: number }[],
  linkIdx: number,
  color: string,
  dark = false,
) {
  const ctx = canvas.getContext("2d");
  if (!ctx || data.length < 2) return;
  const w = canvas.width, h = canvas.height;
  ctx.clearRect(0, 0, w, h);

  const margin = { top: 20, right: 10, bottom: 30, left: 50 };
  const cw = w - margin.left - margin.right;
  const ch = h - margin.top - margin.bottom;

  let tMin = Infinity, tMax = -Infinity, dMin = Infinity, dMax = -Infinity;
  for (const d of data) {
    if (d.theta < tMin) tMin = d.theta;
    if (d.theta > tMax) tMax = d.theta;
    if (d.thetaDot < dMin) dMin = d.thetaDot;
    if (d.thetaDot > dMax) dMax = d.thetaDot;
  }
  const tRange = Math.max(tMax - tMin, 0.01) * 1.1;
  const dRange = Math.max(dMax - dMin, 0.01) * 1.1;
  const tMid = (tMax + tMin) / 2, dMid = (dMax + dMin) / 2;

  ctx.strokeStyle = dark ? "#3a3a5a" : "#ccc";
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.moveTo(margin.left, margin.top);
  ctx.lineTo(margin.left, margin.top + ch);
  ctx.lineTo(margin.left + cw, margin.top + ch);
  ctx.stroke();

  ctx.beginPath();
  ctx.strokeStyle = color;
  ctx.lineWidth = 1.5;
  ctx.globalAlpha = 0.7;
  for (let i = 0; i < data.length; i++) {
    const x = margin.left + ((data[i].theta - tMid + tRange / 2) / tRange) * cw;
    const y = margin.top + ch - ((data[i].thetaDot - dMid + dRange / 2) / dRange) * ch;
    if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
  }
  ctx.stroke();
  ctx.globalAlpha = 1.0;

  const last = data[data.length - 1];
  ctx.beginPath();
  ctx.fillStyle = color;
  const lx = margin.left + ((last.theta - tMid + tRange / 2) / tRange) * cw;
  const ly = margin.top + ch - ((last.thetaDot - dMid + dRange / 2) / dRange) * ch;
  ctx.arc(lx, ly, 4, 0, 2 * Math.PI);
  ctx.fill();

  ctx.fillStyle = dark ? "#8888aa" : "#666";
  ctx.font = "11px monospace";
  ctx.fillText(`θ${linkIdx + 1} vs θ̇${linkIdx + 1}`, margin.left + 4, margin.top - 6);
}

function App() {
  const [systems, setSystems] = useState<PendulumSystem[]>(DEFAULT_SYSTEMS);
  const [g, setG] = useState(9.80665);
  const [playing, setPlaying] = useState(false);
  const [speed, setSpeed] = useState(1);
  const [selectedSystem, setSelectedSystem] = useState(0);
  const [displayStates, setDisplayStates] = useState<PendulumState[]>([EMPTY_STATE]);
  const [loading, setLoading] = useState(true);
  const [fps, setFps] = useState(0);
  const { theme, toggle: toggleTheme } = useTheme();
  const dark = theme === "dark";

  const windowWidth = useWindowWidth();
  const mobile = windowWidth < MOBILE_BREAKPOINT;

  const pendW = mobile ? Math.min(windowWidth - 24, 400) : 500;
  const pendH = mobile ? Math.round(pendW * 0.8) : 400;
  const energyW = mobile ? pendW : 340;
  const energyH = mobile ? 120 : 160;
  const phaseW = mobile ? Math.floor((pendW - 8) / 2) : 280;
  const phaseH = mobile ? Math.round(phaseW * 0.7) : 200;

  const enginesRef = useRef<PendulumEngine[]>([]);
  const animRef = useRef(0);
  const pendCanvasRef = useRef<HTMLCanvasElement>(null);
  const energyCanvasRef = useRef<HTMLCanvasElement>(null);
  const phaseCanvasRefs = useRef<(HTMLCanvasElement | null)[]>([null, null, null]);
  const dataRef = useRef<PerSystemData[]>([]);
  const frameRef = useRef(0);
  const fpsCounter = useRef({ frames: 0, lastTime: performance.now() });

  const systemsRef = useRef(systems);
  const speedRef = useRef(speed);
  const selectedRef = useRef(selectedSystem);
  const darkRef = useRef(dark);
  systemsRef.current = systems;
  speedRef.current = speed;
  selectedRef.current = selectedSystem;
  darkRef.current = dark;

  const drawAll = useCallback((states: PendulumState[], drawPlots = true) => {
    const syss = systemsRef.current;
    const allLinks = syss.map(s => s.links.slice(0, s.N));
    const activeStates = states.map((s, i) => ({
      ...s,
      theta: s.theta.slice(0, syss[i].N),
      theta_dot: s.theta_dot.slice(0, syss[i].N),
      positions: s.positions.slice(0, syss[i].N),
    }));
    const trails = dataRef.current.map(d => d.trail);

    const dk = darkRef.current;
    if (pendCanvasRef.current)
      drawPendulumCanvas(pendCanvasRef.current, activeStates, allLinks, trails, dk);
    if (!drawPlots) return;
    if (energyCanvasRef.current)
      drawEnergyCanvas(energyCanvasRef.current, dataRef.current.map(d => d.energy), dk);

    const si = selectedRef.current;
    if (si < syss.length) {
      const N = syss[si].N;
      const color = SYSTEM_COLORS[si];
      for (let i = 0; i < N; i++) {
        const c = phaseCanvasRefs.current[i];
        if (c) drawPhaseCanvas(c, dataRef.current[si]?.phase[i] || [], i, color, dk);
      }
    }
  }, []);

  const stepOnce = useCallback(() => {
    const engines = enginesRef.current;
    if (engines.length === 0) return;

    const states: PendulumState[] = [];
    for (let si = 0; si < engines.length; si++) {
      engines[si].advance(speedRef.current / 60);
      const s = engines[si].getState();
      states.push(s);

      const d = dataRef.current[si];
      if (s.positions.length > 0) {
        d.trail.push(s.positions[s.positions.length - 1]);
        if (d.trail.length > TRAIL_MAX) d.trail.shift();
      }
      d.energy.push({ t: s.t, dE: s.energy - d.e0 });
      if (d.energy.length > ENERGY_MAX) d.energy.shift();
      for (let i = 0; i < s.theta.length; i++) {
        d.phase[i].push({ theta: s.theta[i], thetaDot: s.theta_dot[i] });
        if (d.phase[i].length > PHASE_MAX) d.phase[i].shift();
      }
    }

    frameRef.current++;
    drawAll(states, frameRef.current % 3 === 0);

    if (frameRef.current % 6 === 0) {
      const syss = systemsRef.current;
      setDisplayStates(states.map((s, i) => ({
        ...s,
        theta: s.theta.slice(0, syss[i].N),
        theta_dot: s.theta_dot.slice(0, syss[i].N),
        positions: s.positions.slice(0, syss[i].N),
      })));
    }
  }, [drawAll]);

  const resetEngines = useCallback(async () => {
    cancelAnimationFrame(animRef.current);
    setPlaying(false);

    for (const e of enginesRef.current) e.destroy();
    enginesRef.current = [];
    dataRef.current = [];
    frameRef.current = 0;

    const engines: PendulumEngine[] = [];
    const newData: PerSystemData[] = [];
    const initStates: PendulumState[] = [];

    for (const sys of systems) {
      const engine = new PendulumEngine(
        sys.links.slice(0, sys.N),
        sys.theta0.slice(0, sys.N),
        sys.thetaDot0.slice(0, sys.N),
        g,
      );
      try {
        await engine.init();
        engines.push(engine);
        const s = engine.getState();
        initStates.push(s);
        newData.push({
          trail: [], energy: [],
          phase: [[], [], []], e0: s.energy,
        });
      } catch (err) {
        console.error("Failed to init WASM engine:", err);
      }
    }

    enginesRef.current = engines;
    dataRef.current = newData;

    const syss = systems;
    setDisplayStates(initStates.map((s, i) => ({
      ...s,
      theta: s.theta.slice(0, syss[i].N),
      theta_dot: s.theta_dot.slice(0, syss[i].N),
      positions: s.positions.slice(0, syss[i].N),
    })));
    setLoading(false);
    requestAnimationFrame(() => drawAll(initStates));
  }, [systems, g, drawAll]);

  useEffect(() => {
    resetEngines();
    return () => {
      cancelAnimationFrame(animRef.current);
      for (const e of enginesRef.current) e.destroy();
    };
  }, [resetEngines]);

  useEffect(() => {
    if (!playing) {
      cancelAnimationFrame(animRef.current);
      return;
    }
    const loop = () => {
      stepOnce();
      fpsCounter.current.frames++;
      const now = performance.now();
      if (now - fpsCounter.current.lastTime >= 1000) {
        setFps(fpsCounter.current.frames);
        fpsCounter.current.frames = 0;
        fpsCounter.current.lastTime = now;
      }
      animRef.current = requestAnimationFrame(loop);
    };
    animRef.current = requestAnimationFrame(loop);
    return () => cancelAnimationFrame(animRef.current);
  }, [playing, stepOnce]);

  if (loading) {
    return (
      <div style={{ display: "flex", justifyContent: "center", alignItems: "center", height: "100vh", fontFamily: "system-ui" }}>
        Loading WASM module...
      </div>
    );
  }

  const selSys = systems[selectedSystem] || systems[0];

  const canvasBg = dark ? "#16213e" : "#fafafa";
  const panelBg = dark ? "#1a1a2e" : "#f9f9f9";
  const borderColor = dark ? "#2a2a4a" : "#eee";
  const mutedColor = dark ? "#8888aa" : "#888";

  // --- Mobile layout ---
  if (mobile) {
    return (
      <div style={{ fontFamily: "system-ui", height: "100vh", display: "flex", flexDirection: "column", background: "var(--bg)", color: "var(--fg)" }}>
        {/* Fixed top: playback + pendulum */}
        <div style={{ flexShrink: 0 }}>
          <div style={{
            display: "flex", gap: 6, padding: "8px 12px", borderBottom: `1px solid ${borderColor}`,
            alignItems: "center", background: "var(--bg)",
          }}>
            <button onClick={() => setPlaying(p => !p)}
              style={{ flex: 1, padding: "10px 0", fontSize: 16, cursor: "pointer" }}>
              {playing ? "⏸ Pause" : "▶ Play"}
            </button>
            <button onClick={stepOnce} style={{ padding: "10px 14px", cursor: "pointer" }}>Step</button>
            <button onClick={resetEngines} style={{ padding: "10px 14px", cursor: "pointer" }}>Reset</button>
            <button onClick={toggleTheme} style={{ padding: "10px 14px", cursor: "pointer" }}>
              {dark ? "☀" : "☾"}
            </button>
          </div>
          <div style={{ padding: "4px 12px" }}>
            <canvas ref={pendCanvasRef} width={pendW} height={pendH}
              style={{ background: canvasBg, borderRadius: 8, width: "100%" }} />
          </div>
        </div>

        {/* Scrollable: info, speed, graphs, parameters */}
        <div style={{ flex: 1, overflow: "auto", WebkitOverflowScrolling: "touch" }}>
          {/* Info row */}
          <div style={{ display: "flex", gap: 4, padding: "6px 12px", flexWrap: "wrap" }}>
            {displayStates.map((s, i) => (
              <div key={i} style={{
                flex: 1, minWidth: 100, padding: 6, background: panelBg, borderRadius: 4,
                fontFamily: "monospace", fontSize: 10, lineHeight: 1.3,
                borderLeft: `3px solid ${SYSTEM_COLORS[i]}`,
              }}>
                <div><strong>S{i + 1}</strong> t={s.t.toFixed(1)}s</div>
                <div>|dE/E₀|={s.energy_drift.toExponential(1)}</div>
              </div>
            ))}
            <div style={{ padding: 6, fontFamily: "monospace", fontSize: 10 }}>FPS={fps}</div>
          </div>

          {/* Speed */}
          <div style={{ padding: "2px 12px", fontSize: 12 }}>
            Speed: {speed >= 1 ? speed + "x" : "1/" + (1 / speed) + "x"}
            <input type="range" min="-3" max="3" step="1"
              value={Math.round(Math.log2(speed))}
              onChange={(e) => setSpeed(Math.pow(2, +e.target.value))}
              style={{ width: "100%", margin: "4px 0" }} />
          </div>

          {/* Energy plot */}
          <div style={{ padding: "4px 12px" }}>
            <canvas ref={energyCanvasRef} width={energyW} height={energyH}
              style={{ background: canvasBg, borderRadius: 6, width: "100%" }} />
          </div>

          {/* Phase space */}
          <div style={{ padding: "4px 12px" }}>
            <div style={{ fontSize: 11, color: mutedColor, marginBottom: 4 }}>
              Phase space (System {selectedSystem + 1})
            </div>
            <div style={{ display: "flex", gap: 4 }}>
              {Array.from({ length: selSys.N }, (_, i) => (
                <canvas key={`${selectedSystem}-${i}`}
                  ref={(el) => { phaseCanvasRefs.current[i] = el; }}
                  width={phaseW} height={phaseH}
                  style={{ background: canvasBg, borderRadius: 4, flex: 1 }} />
              ))}
            </div>
          </div>

          {/* Parameters inline */}
          <div style={{ padding: "8px 12px" }}>
            <Controls
              systems={systems} g={g} playing={playing} speed={speed}
              selectedSystem={selectedSystem}
              onSystemsChange={setSystems} onGChange={setG}
              onPlayPause={() => setPlaying(p => !p)}
              onReset={resetEngines} onStep={stepOnce}
              onSpeedChange={setSpeed}
              onSelectedSystemChange={setSelectedSystem}
              hidePlayback
            />
          </div>

          <div style={{ height: 40 }} />
        </div>
      </div>
    );
  }

  // --- Desktop layout ---
  return (
    <div style={{ display: "flex", fontFamily: "system-ui", height: "100vh", overflow: "hidden", background: "var(--bg)", color: "var(--fg)" }}>
      <Controls
        systems={systems} g={g} playing={playing} speed={speed}
        selectedSystem={selectedSystem}
        onSystemsChange={setSystems} onGChange={setG}
        onPlayPause={() => setPlaying(p => !p)}
        onReset={resetEngines} onStep={stepOnce}
        onSpeedChange={setSpeed}
        onSelectedSystemChange={setSelectedSystem}
      />

      <div style={{ flex: 1, display: "flex", flexDirection: "column", gap: 8, padding: 16, overflow: "auto" }}>
        <div style={{ display: "flex", gap: 16 }}>
          <canvas ref={pendCanvasRef} width={pendW} height={pendH}
            style={{ background: canvasBg, borderRadius: 8 }} />
          <div style={{ display: "flex", flexDirection: "column", gap: 8 }}>
            <div style={{ display: "flex", flexDirection: "column", gap: 4 }}>
              {displayStates.map((s, i) => (
                <div key={i} style={{
                  padding: 8, background: panelBg, borderRadius: 6,
                  fontFamily: "monospace", fontSize: 11, lineHeight: 1.4,
                  borderLeft: `3px solid ${SYSTEM_COLORS[i]}`,
                  opacity: i === selectedSystem ? 1 : 0.7,
                }}>
                  <div><strong>S{i + 1}</strong> t={s.t.toFixed(2)}s E={s.energy.toFixed(4)}J |dE/E0|={s.energy_drift.toExponential(2)}</div>
                </div>
              ))}
              <div style={{ padding: 4, fontFamily: "monospace", fontSize: 12, display: "flex", alignItems: "center", gap: 8 }}>
                <span><strong>FPS</strong> = {fps}</span>
                <button onClick={toggleTheme}
                  style={{ marginLeft: "auto", padding: "2px 8px", cursor: "pointer", fontSize: 14 }}>
                  {dark ? "☀" : "☾"}
                </button>
              </div>
            </div>
            <canvas ref={energyCanvasRef} width={energyW} height={energyH}
              style={{ background: canvasBg, borderRadius: 6 }} />
          </div>
        </div>

        <div style={{ fontSize: 12, color: mutedColor, marginTop: 4 }}>
          Phase space (System {selectedSystem + 1})
        </div>
        <div style={{ display: "flex", gap: 8, flexWrap: "wrap" }}>
          {Array.from({ length: selSys.N }, (_, i) => (
            <canvas key={`${selectedSystem}-${i}`}
              ref={(el) => { phaseCanvasRefs.current[i] = el; }}
              width={phaseW} height={phaseH}
              style={{ background: canvasBg, borderRadius: 6 }} />
          ))}
        </div>
      </div>
    </div>
  );
}

export default App;
