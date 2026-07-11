// Occupancy grid renderer: paints free/occupied/unknown cells, overlays beam
// hits, the corrected pose, an optional raw-EKF pose, and a trajectory trail.
import { COLORS } from "./constants.js";

function decodeMapCells(payload) {
  if (payload.data_b64) {
    const raw = atob(payload.data_b64);
    const cells = new Int8Array(raw.length);
    for (let i = 0; i < raw.length; i++) cells[i] = raw.charCodeAt(i);
    return cells;
  }
  if (Array.isArray(payload.data)) return payload.data;
  return null;
}

export function drawMap(p) {
  const info = document.getElementById("map-info");
  const cv = document.getElementById("map");
  const ctx = cv.getContext("2d");
  const { width, height, res, origin, pose, pose_raw, trail, points } = p;
  const data = decodeMapCells(p);
  if (!width || !height || !data || data.length < width * height) {
    if (info) info.textContent = "map frame invalid";
    return;
  }

  try {
    // Fit the grid into the canvas preserving aspect ratio.
    const scale = Math.min(cv.width / width, cv.height / height);
    const offX = (cv.width - width * scale) / 2;
    const offY = (cv.height - height * scale) / 2;

    ctx.fillStyle = COLORS.bg;
    ctx.fillRect(0, 0, cv.width, cv.height);

    const img = ctx.createImageData(width, height);
    const limit = width * height;
    for (let i = 0; i < limit; i++) {
      const v = data[i];
      let r, g, b;
      if (v < 0) { r = 49; g = 49; b = 49; }                         // unknown
      else if (v < 50) { r = 26; g = 58; b = 38; }                  // free
      else { r = 248; g = 81; b = 73; }                             // occupied
      const j = i * 4;
      img.data[j] = r; img.data[j + 1] = g; img.data[j + 2] = b; img.data[j + 3] = 255;
    }
    // Render grid to an offscreen canvas, then scale onto the visible one.
    const off = document.createElement("canvas");
    off.width = width; off.height = height;
    off.getContext("2d").putImageData(img, 0, 0);
    ctx.imageSmoothingEnabled = false;
    // Flip vertically so +y (world) points up on screen.
    ctx.save();
    ctx.translate(offX, offY + height * scale);
    ctx.scale(scale, -scale);
    ctx.drawImage(off, 0, 0);
    ctx.restore();

    // World -> canvas pixel mapping (y flipped).
    const worldToPx = (wx, wy) => {
      const gx = (wx - origin.x) / res;
      const gy = (wy - origin.y) / res;
      return [offX + gx * scale, offY + (height - gy) * scale];
    };

    // Beam endpoints
    if (points && points.length) {
      ctx.fillStyle = COLORS.hit;
      for (const [wx, wy] of points) {
        const [px, py] = worldToPx(wx, wy);
        ctx.fillRect(px - 1, py - 1, 2, 2);
      }
    }

    // Corrected trajectory trail
    if (trail && trail.length) {
      ctx.strokeStyle = COLORS.success; ctx.lineWidth = 1.5;
      ctx.beginPath();
      trail.forEach(([wx, wy], i) => {
        const [px, py] = worldToPx(wx, wy);
        if (i === 0) ctx.moveTo(px, py); else ctx.lineTo(px, py);
      });
      ctx.stroke();
    }

    // Raw (uncorrected) EKF pose, shown to visualize scan-match drift correction
    if (pose_raw) {
      const [px, py] = worldToPx(pose_raw.x, pose_raw.y);
      ctx.fillStyle = COLORS.warning;
      ctx.beginPath(); ctx.arc(px, py, 4, 0, Math.PI * 2); ctx.fill();
    }

    // Drone pose (corrected)
    if (pose) {
      const [px, py] = worldToPx(pose.x, pose.y);
      ctx.fillStyle = COLORS.accent;
      ctx.beginPath(); ctx.arc(px, py, 5, 0, Math.PI * 2); ctx.fill();
      ctx.strokeStyle = COLORS.accent; ctx.lineWidth = 2;
      ctx.beginPath(); ctx.moveTo(px, py);
      ctx.lineTo(px + Math.cos(pose.yaw) * 14, py - Math.sin(pose.yaw) * 14);
      ctx.stroke();
    }

    if (info) {
      info.textContent =
        `${width}x${height} cells @ ${res.toFixed(2)} m  (${(width * res).toFixed(1)} x ${(height * res).toFixed(1)} m)`;
    }
  } catch (err) {
    console.error("map render failed", err);
    if (info) info.textContent = "map render failed";
  }
}
