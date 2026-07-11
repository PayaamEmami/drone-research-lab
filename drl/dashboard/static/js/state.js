// State-estimate readout (x/y/z, roll/pitch/yaw) and the commanded body-frame
// velocity vector panel.
import { COLORS } from "./constants.js";

export function updateState(p) {
  const root = document.getElementById("state-readout");
  const set = (k, v, deg) => {
    const el = root.querySelector(`[data-k="${k}"]`);
    if (el && v != null) el.textContent = v.toFixed(deg ? 1 : 3);
  };
  set("x", p.x); set("y", p.y); set("z", p.z);
  set("roll", p.roll, true); set("pitch", p.pitch, true); set("yaw", p.yaw, true);
}

export function drawCmd(p) {
  const cv = document.getElementById("cmd");
  const ctx = cv.getContext("2d");
  const W = cv.width, H = cv.height, cx = W / 2, cy = H / 2;
  const scale = (Math.min(W, H) / 2 - 12) / 0.5; // full deflection at 0.5 m/s
  ctx.clearRect(0, 0, W, H);
  ctx.strokeStyle = COLORS.grid;
  ctx.beginPath(); ctx.moveTo(cx, 6); ctx.lineTo(cx, H - 6);
  ctx.moveTo(6, cy); ctx.lineTo(W - 6, cy); ctx.stroke();

  const vx = p.vx || 0, vy = p.vy || 0;
  // body frame: vx forward (up on screen), vy left (left on screen)
  const ex = cx - vy * scale, ey = cy - vx * scale;
  ctx.strokeStyle = COLORS.accent; ctx.fillStyle = COLORS.accent; ctx.lineWidth = 3;
  ctx.beginPath(); ctx.moveTo(cx, cy); ctx.lineTo(ex, ey); ctx.stroke();
  ctx.beginPath(); ctx.arc(ex, ey, 5, 0, Math.PI * 2); ctx.fill();

  document.getElementById("cmd-readout").textContent =
    `${p.label ? p.label + " | " : ""}vx ${vx.toFixed(2)}, vy ${vy.toFixed(2)}`;
}
