import { useRef, useEffect } from "react";

interface Props {
  data: { t: number; dE: number }[];
  width: number;
  height: number;
}

export function EnergyPlot({ data, width, height }: Props) {
  const canvasRef = useRef<HTMLCanvasElement>(null);

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas || data.length < 2) return;
    const ctx = canvas.getContext("2d");
    if (!ctx) return;

    ctx.clearRect(0, 0, width, height);

    const margin = { top: 20, right: 10, bottom: 30, left: 60 };
    const w = width - margin.left - margin.right;
    const h = height - margin.top - margin.bottom;

    const tMax = data[data.length - 1].t || 1;
    const absMax =
      Math.max(...data.map((d) => Math.abs(d.dE)), 1e-15) * 1.2;

    ctx.save();
    ctx.translate(margin.left, margin.top);

    // Axes
    ctx.strokeStyle = "#ccc";
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(0, 0);
    ctx.lineTo(0, h);
    ctx.lineTo(w, h);
    ctx.stroke();

    // Zero line
    const y0 = h / 2;
    ctx.strokeStyle = "#ddd";
    ctx.setLineDash([4, 4]);
    ctx.beginPath();
    ctx.moveTo(0, y0);
    ctx.lineTo(w, y0);
    ctx.stroke();
    ctx.setLineDash([]);

    // Data line
    ctx.beginPath();
    ctx.strokeStyle = "#e74c3c";
    ctx.lineWidth = 1.5;
    for (let i = 0; i < data.length; i++) {
      const x = (data[i].t / tMax) * w;
      const y = y0 - (data[i].dE / absMax) * (h / 2);
      if (i === 0) ctx.moveTo(x, y);
      else ctx.lineTo(x, y);
    }
    ctx.stroke();

    // Labels
    ctx.restore();
    ctx.fillStyle = "#666";
    ctx.font = "11px monospace";
    ctx.fillText("E(t) − E₀", margin.left + 4, margin.top - 6);
    ctx.fillText(`t = ${tMax.toFixed(1)}s`, width - margin.right - 60, height - 6);
    ctx.fillText(`±${absMax.toExponential(1)}`, 2, margin.top + h / 2 - 4);
  }, [data, width, height]);

  return (
    <canvas
      ref={canvasRef}
      width={width}
      height={height}
      style={{ background: "#fafafa", borderRadius: 6 }}
    />
  );
}
