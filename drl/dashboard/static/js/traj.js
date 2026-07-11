// Trajectory-tracking view: top-down reference path vs. estimate, plus command.
// Frame: traj -> { reference:{x,y,z}, estimate:{x,y,z}, command:{vx,vy,vz} }
import { COLORS } from "./constants.js";

const refPath = [];
const estPath = [];
const MAX_PATH = 600;

function fitBounds(pts) {
  let min = -1, max = 1;
  for (const [x, y] of pts) {
    min = Math.min(min, x, y);
    max = Math.max(max, x, y);
  }
  const pad = (max - min) * 0.1 || 0.5;
  return [min - pad, max + pad];
}

export function drawTraj(p) {
  const cv = document.getElementById("traj");
  if (!cv) return;
  const ctx = cv.getContext("2d");
  const W = cv.width, H = cv.height;
  const ref = p.reference || {}, est = p.estimate || {}, cmd = p.command || {};

  if (ref.x != null) refPath.push([ref.x, ref.y]);
  if (est.x != null) estPath.push([est.x, est.y]);
  if (refPath.length > MAX_PATH) refPath.shift();
  if (estPath.length > MAX_PATH) estPath.shift();

  const [lo, hi] = fitBounds(refPath.concat(estPath));
  const span = hi - lo || 1;
  const toPx = (x, y) => [((x - lo) / span) * W, H - ((y - lo) / span) * H];

  ctx.fillStyle = COLORS.bg;
  ctx.fillRect(0, 0, W, H);

  const drawPath = (pts, color, dash) => {
    if (!pts.length) return;
    ctx.strokeStyle = color; ctx.lineWidth = 2; ctx.setLineDash(dash);
    ctx.beginPath();
    pts.forEach(([x, y], i) => {
      const [px, py] = toPx(x, y);
      if (i === 0) ctx.moveTo(px, py); else ctx.lineTo(px, py);
    });
    ctx.stroke();
    ctx.setLineDash([]);
  };

  drawPath(refPath, COLORS.muted, [5, 4]);
  drawPath(estPath, COLORS.accent, []);

  // Current estimate marker + command arrow (world frame).
  if (est.x != null) {
    const [px, py] = toPx(est.x, est.y);
    ctx.fillStyle = COLORS.accent;
    ctx.beginPath(); ctx.arc(px, py, 5, 0, Math.PI * 2); ctx.fill();
    if (cmd.vx != null) {
      const s = 40;
      ctx.strokeStyle = COLORS.left; ctx.lineWidth = 2;
      ctx.beginPath(); ctx.moveTo(px, py);
      ctx.lineTo(px + cmd.vx * s, py - cmd.vy * s); ctx.stroke();
    }
  }

  const info = document.getElementById("traj-info");
  if (info) {
    info.textContent =
      `ref (${(ref.x ?? 0).toFixed(2)}, ${(ref.y ?? 0).toFixed(2)}, ${(ref.z ?? 0).toFixed(2)})  ` +
      `est (${(est.x ?? 0).toFixed(2)}, ${(est.y ?? 0).toFixed(2)}, ${(est.z ?? 0).toFixed(2)})`;
  }
}
