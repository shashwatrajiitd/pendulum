import { useRef, useEffect } from "react";
import type { PendulumState, LinkConfig } from "../engine/wasm-bridge";

interface Props {
  state: PendulumState;
  links: LinkConfig[];
  trail: { x: number; y: number }[];
  width: number;
  height: number;
}

const COLORS = ["#e74c3c", "#3498db", "#2ecc71"];

export function PendulumCanvas({ state, links, trail, width, height }: Props) {
  const canvasRef = useRef<HTMLCanvasElement>(null);

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    const ctx = canvas.getContext("2d");
    if (!ctx) return;

    const scale =
      (Math.min(width, height) * 0.35) /
      links.reduce((sum, l) => sum + l.L, 0);
    const ox = width / 2;
    const oy = height * 0.35;

    ctx.clearRect(0, 0, width, height);

    // Draw trail (last bob)
    if (trail.length > 1) {
      ctx.beginPath();
      ctx.strokeStyle = "rgba(231, 76, 60, 0.3)";
      ctx.lineWidth = 1.5;
      for (let i = 0; i < trail.length; i++) {
        const sx = ox + trail[i].x * scale;
        const sy = oy - trail[i].y * scale;
        if (i === 0) ctx.moveTo(sx, sy);
        else ctx.lineTo(sx, sy);
      }
      ctx.stroke();
    }

    // Draw pivot
    ctx.fillStyle = "#333";
    ctx.beginPath();
    ctx.arc(ox, oy, 4, 0, 2 * Math.PI);
    ctx.fill();

    // Draw rods and bobs
    let prevX = ox;
    let prevY = oy;

    for (let i = 0; i < state.positions.length; i++) {
      const bx = ox + state.positions[i].x * scale;
      const by = oy - state.positions[i].y * scale;

      // Rod
      ctx.beginPath();
      ctx.strokeStyle = "#555";
      ctx.lineWidth = 2.5;
      ctx.moveTo(prevX, prevY);
      ctx.lineTo(bx, by);
      ctx.stroke();

      // Bob
      const bobRadius = Math.max(links[i].r_bob * scale, 6);
      ctx.beginPath();
      ctx.fillStyle = COLORS[i % COLORS.length];
      ctx.arc(bx, by, bobRadius, 0, 2 * Math.PI);
      ctx.fill();
      ctx.strokeStyle = "#333";
      ctx.lineWidth = 1;
      ctx.stroke();

      prevX = bx;
      prevY = by;
    }
  }, [state, links, trail, width, height]);

  return (
    <canvas
      ref={canvasRef}
      width={width}
      height={height}
      style={{ background: "#fafafa", borderRadius: 8 }}
    />
  );
}
