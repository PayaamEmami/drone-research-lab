// Radial proximity HUD: draws the four horizontal beams as spokes (length =
// distance, color graded near=red -> far=green) plus an up/down readout.
import { DISPLAY_RANGE_M, DIRS, COLORS } from "./constants.js";

export function proximityColor(d) {
  if (d == null) return COLORS.surfaceInput;
  const t = Math.max(0, Math.min(1, d / DISPLAY_RANGE_M));
  // near = error, far = success
  const r = Math.round(248 * (1 - t) + 46 * t);
  const g = Math.round(81 * (1 - t) + 160 * t);
  const b = Math.round(73 * (1 - t) + 67 * t);
  return `rgb(${r},${g},${b})`;
}

export function drawHud(p) {
  const cv = document.getElementById("hud");
  const ctx = cv.getContext("2d");
  const W = cv.width, H = cv.height;
  const cx = W / 2, cy = H / 2;
  const maxR = Math.min(W, H) / 2 - 24;

  ctx.clearRect(0, 0, W, H);

  // Range rings
  ctx.strokeStyle = COLORS.grid;
  ctx.fillStyle = COLORS.muted;
  ctx.font = "10px system-ui";
  ctx.textAlign = "center";
  for (let i = 1; i <= 4; i++) {
    const rr = (maxR * i) / 4;
    ctx.beginPath();
    ctx.arc(cx, cy, rr, 0, Math.PI * 2);
    ctx.stroke();
    ctx.fillText(((DISPLAY_RANGE_M * i) / 4).toFixed(1) + "m", cx, cy - rr - 3);
  }

  // Direction unit vectors on the canvas (front=up, screen y is down)
  const vec = {
    front: [0, -1],
    back: [0, 1],
    left: [-1, 0],
    right: [1, 0],
  };

  for (const dir of DIRS) {
    const d = p[dir];
    const [ux, uy] = vec[dir];
    const r = d == null ? maxR : (Math.min(d, DISPLAY_RANGE_M) / DISPLAY_RANGE_M) * maxR;
    const ex = cx + ux * r, ey = cy + uy * r;

    // Beam line
    ctx.strokeStyle = proximityColor(d);
    ctx.lineWidth = d == null ? 1 : 3;
    ctx.globalAlpha = d == null ? 0.35 : 1;
    ctx.beginPath();
    ctx.moveTo(cx, cy);
    ctx.lineTo(ex, ey);
    ctx.stroke();

    // Endpoint dot + label
    if (d != null) {
      ctx.fillStyle = proximityColor(d);
      ctx.beginPath();
      ctx.arc(ex, ey, 5, 0, Math.PI * 2);
      ctx.fill();
    }
    ctx.globalAlpha = 1;
    ctx.fillStyle = COLORS.text;
    ctx.fillText(d == null ? "--" : d.toFixed(2), cx + ux * (maxR + 14), cy + uy * (maxR + 14) + 3);
  }

  // Center body
  ctx.fillStyle = COLORS.accent;
  ctx.beginPath();
  ctx.arc(cx, cy, 7, 0, Math.PI * 2);
  ctx.fill();

  // up/down readout
  const ro = document.getElementById("hud-readout");
  const fmt = (v) => (v == null ? "--" : v.toFixed(2) + " m");
  ro.querySelector('[data-dir="up"]').textContent = "up: " + fmt(p.up);
  ro.querySelector('[data-dir="down"]').textContent = "down: " + fmt(p.down);
}
