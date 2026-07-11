// 3-D point cloud rendered as an orthographic scatter (height mapped to color).
// Frame: cloud -> { points: [[x, y, z], ...] }
import { COLORS } from "./constants.js";

let points = [];

// Isometric-ish projection: rotate about vertical, tilt down slightly.
const YAW = Math.PI / 6;
const TILT = 0.5;
const cosY = Math.cos(YAW), sinY = Math.sin(YAW);

function project(x, y, z) {
  const px = x * cosY - y * sinY;
  const py = (x * sinY + y * cosY) * TILT - z;
  return [px, py];
}

export function drawCloud(p) {
  if (Array.isArray(p.points)) points = p.points;
  const cv = document.getElementById("cloud");
  if (!cv) return;
  const ctx = cv.getContext("2d");
  const W = cv.width, H = cv.height;
  ctx.fillStyle = COLORS.bg;
  ctx.fillRect(0, 0, W, H);
  if (!points.length) return;

  const proj = points.map(([x, y, z]) => project(x, y, z));
  let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity;
  let minZ = Infinity, maxZ = -Infinity;
  proj.forEach(([px, py]) => {
    minX = Math.min(minX, px); maxX = Math.max(maxX, px);
    minY = Math.min(minY, py); maxY = Math.max(maxY, py);
  });
  points.forEach(([, , z]) => { minZ = Math.min(minZ, z); maxZ = Math.max(maxZ, z); });

  const spanX = maxX - minX || 1, spanY = maxY - minY || 1;
  const scale = Math.min(W / spanX, H / spanY) * 0.85;
  const offX = (W - spanX * scale) / 2, offY = (H - spanY * scale) / 2;
  const zSpan = maxZ - minZ || 1;

  proj.forEach(([px, py], i) => {
    const z = points[i][2];
    const t = (z - minZ) / zSpan;
    const r = Math.round(0 + t * 77);
    const g = Math.round(120 + t * 50);
    const b = Math.round(212 + t * 40);
    ctx.fillStyle = `rgb(${r},${g},${b})`;
    const sx = offX + (px - minX) * scale;
    const sy = offY + (py - minY) * scale;
    ctx.fillRect(sx, sy, 2, 2);
  });

  const info = document.getElementById("cloud-info");
  if (info) info.textContent = `${points.length} points`;
}
