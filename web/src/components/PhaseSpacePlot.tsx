import { useRef, useEffect } from "react";

interface Props {
  data: { theta: number; thetaDot: number }[];
  linkIdx: number;
  width: number;
  height: number;
}

const COLORS = ["#e74c3c", "#3498db", "#2ecc71"];

export function PhaseSpacePlot({ data, linkIdx, width, height }: Props) {
  const canvasRef = useRef<HTMLCanvasElement>(null);

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas || data.length < 2) return;
    const ctx = canvas.getContext("2d");
    if (!ctx) return;

    ctx.clearRect(0, 0, width, height);

    const margin = { top: 20, right: 10, bottom: 30, left: 50 };
    const w = width - margin.left - margin.right;
    const h = height - margin.top - margin.bottom;

    const thetaVals = data.map((d) => d.theta);
    const dotVals = data.map((d) => d.thetaDot);
    const thetaMin = Math.min(...thetaVals);
    const thetaMax = Math.max(...thetaVals);
    const dotMin = Math.min(...dotVals);
    const dotMax = Math.max(...dotVals);

    const tRange = Math.max(thetaMax - thetaMin, 0.01) * 1.1;
    const dRange = Math.max(dotMax - dotMin, 0.01) * 1.1;
    const tMid = (thetaMax + thetaMin) / 2;
    const dMid = (dotMax + dotMin) / 2;

    const toX = (t: number) => margin.left + ((t - tMid + tRange / 2) / tRange) * w;
    const toY = (d: number) => margin.top + h - ((d - dMid + dRange / 2) / dRange) * h;

    // Axes
    ctx.strokeStyle = "#ccc";
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(margin.left, margin.top);
    ctx.lineTo(margin.left, margin.top + h);
    ctx.lineTo(margin.left + w, margin.top + h);
    ctx.stroke();

    // Trajectory
    ctx.beginPath();
    ctx.strokeStyle = COLORS[linkIdx % COLORS.length];
    ctx.lineWidth = 1.5;
    ctx.globalAlpha = 0.7;
    for (let i = 0; i < data.length; i++) {
      const x = toX(data[i].theta);
      const y = toY(data[i].thetaDot);
      if (i === 0) ctx.moveTo(x, y);
      else ctx.lineTo(x, y);
    }
    ctx.stroke();
    ctx.globalAlpha = 1.0;

    // Current point
    if (data.length > 0) {
      const last = data[data.length - 1];
      ctx.beginPath();
      ctx.fillStyle = COLORS[linkIdx % COLORS.length];
      ctx.arc(toX(last.theta), toY(last.thetaDot), 4, 0, 2 * Math.PI);
      ctx.fill();
    }

    // Labels
    ctx.fillStyle = "#666";
    ctx.font = "11px monospace";
    ctx.fillText(`θ${linkIdx + 1} vs θ̇${linkIdx + 1}`, margin.left + 4, margin.top - 6);
  }, [data, linkIdx, width, height]);

  return (
    <canvas
      ref={canvasRef}
      width={width}
      height={height}
      style={{ background: "#fafafa", borderRadius: 6 }}
    />
  );
}
