import { useState, useRef, useCallback, useEffect } from "react";
import { PendulumEngine, type LinkConfig, type PendulumState } from "./engine/wasm-bridge";
import { PendulumCanvas } from "./components/PendulumCanvas";
import { Controls } from "./components/Controls";
import { InfoPanel } from "./components/InfoPanel";
import { EnergyPlot } from "./components/EnergyPlot";
import { PhaseSpacePlot } from "./components/PhaseSpacePlot";

const DEFAULT_LINKS: LinkConfig[] = [
  { L: 1.0, m_rod: 0, m_bob: 1.0, r_bob: 0.05 },
  { L: 1.0, m_rod: 0, m_bob: 1.0, r_bob: 0.05 },
  { L: 1.0, m_rod: 0, m_bob: 1.0, r_bob: 0.05 },
];

const EMPTY_STATE: PendulumState = {
  theta: [0],
  theta_dot: [0],
  t: 0,
  energy: 0,
  energy_drift: 0,
  positions: [{ x: 0, y: 0 }],
};

const TRAIL_MAX = 800;
const ENERGY_MAX = 600;
const PHASE_MAX = 600;

function App() {
  const [N, setN] = useState(2);
  const [links, setLinks] = useState<LinkConfig[]>(DEFAULT_LINKS);
  const [g, setG] = useState(9.80665);
  const [theta0, setTheta0] = useState([2.0, 2.0, 0.5]);
  const [thetaDot0, setThetaDot0] = useState([0, 0, 0]);
  const [playing, setPlaying] = useState(false);
  const [state, setState] = useState<PendulumState>(EMPTY_STATE);
  const [loading, setLoading] = useState(true);
  const [fps, setFps] = useState(0);

  const engineRef = useRef<PendulumEngine | null>(null);
  const animRef = useRef<number>(0);
  const trailRef = useRef<{ x: number; y: number }[]>([]);
  const energyRef = useRef<{ t: number; dE: number }[]>([]);
  const phaseRef = useRef<{ theta: number; thetaDot: number }[][]>([[], [], []]);
  const e0Ref = useRef(0);
  const fpsCounter = useRef({ frames: 0, lastTime: performance.now() });

  const [trail, setTrail] = useState<{ x: number; y: number }[]>([]);
  const [energyData, setEnergyData] = useState<{ t: number; dE: number }[]>([]);
  const [phaseData, setPhaseData] = useState<{ theta: number; thetaDot: number }[][]>([[]]);

  const resetEngine = useCallback(async () => {
    if (engineRef.current) {
      engineRef.current.destroy();
      engineRef.current = null;
    }

    trailRef.current = [];
    energyRef.current = [];
    phaseRef.current = [[], [], []];
    setTrail([]);
    setEnergyData([]);
    setPhaseData([[], [], []]);
    setPlaying(false);

    const engine = new PendulumEngine(
      links.slice(0, N),
      theta0.slice(0, N),
      thetaDot0.slice(0, N),
      g
    );

    try {
      await engine.init();
      engineRef.current = engine;
      const s = engine.getState();
      e0Ref.current = s.energy;
      setState(s);
      setLoading(false);
    } catch (err) {
      console.error("Failed to init WASM engine:", err);
    }
  }, [N, links, g, theta0, thetaDot0]);

  useEffect(() => {
    resetEngine();
    return () => {
      if (engineRef.current) engineRef.current.destroy();
    };
  }, []);

  const stepOnce = useCallback(() => {
    const engine = engineRef.current;
    if (!engine) return;

    engine.advance(1 / 60);
    const s = engine.getState();
    setState(s);

    // Trail (last bob)
    if (s.positions.length > 0) {
      const last = s.positions[s.positions.length - 1];
      trailRef.current.push(last);
      if (trailRef.current.length > TRAIL_MAX) trailRef.current.shift();
      setTrail([...trailRef.current]);
    }

    // Energy plot
    energyRef.current.push({ t: s.t, dE: s.energy - e0Ref.current });
    if (energyRef.current.length > ENERGY_MAX) energyRef.current.shift();
    setEnergyData([...energyRef.current]);

    // Phase space
    for (let i = 0; i < s.theta.length; i++) {
      phaseRef.current[i].push({ theta: s.theta[i], thetaDot: s.theta_dot[i] });
      if (phaseRef.current[i].length > PHASE_MAX) phaseRef.current[i].shift();
    }
    setPhaseData(phaseRef.current.map((a) => [...a]));
  }, []);

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

  return (
    <div style={{ display: "flex", fontFamily: "system-ui", height: "100vh", overflow: "hidden" }}>
      <Controls
        N={N} links={links} g={g} theta0={theta0} thetaDot0={thetaDot0}
        playing={playing}
        onNChange={setN} onLinksChange={setLinks} onGChange={setG}
        onTheta0Change={setTheta0} onThetaDot0Change={setThetaDot0}
        onPlayPause={() => setPlaying((p) => !p)}
        onReset={resetEngine}
        onStep={stepOnce}
      />

      <div style={{ flex: 1, display: "flex", flexDirection: "column", gap: 8, padding: 16, overflow: "auto" }}>
        <div style={{ display: "flex", gap: 16 }}>
          <PendulumCanvas state={state} links={links.slice(0, N)} trail={trail} width={500} height={400} />
          <div style={{ display: "flex", flexDirection: "column", gap: 8 }}>
            <InfoPanel state={state} fps={fps} />
            <EnergyPlot data={energyData} width={340} height={160} />
          </div>
        </div>

        <div style={{ display: "flex", gap: 8, flexWrap: "wrap" }}>
          {Array.from({ length: N }, (_, i) => (
            <PhaseSpacePlot key={i} data={phaseData[i] || []} linkIdx={i} width={280} height={200} />
          ))}
        </div>
      </div>
    </div>
  );
}

export default App;
