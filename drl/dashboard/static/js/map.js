// Occupancy grid renderer: paints free/occupied/unknown cells, overlays the
// latest beam hit points and the drone pose, and reports map dimensions.

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
  const { width, height, res, origin, pose, points } = p;
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

    ctx.fillStyle = "#0e1116";
    ctx.fillRect(0, 0, cv.width, cv.height);

    const img = ctx.createImageData(width, height);
    const limit = width * height;
    for (let i = 0; i < limit; i++) {
      const v = data[i];
      let r, g, b;
      if (v < 0) { r = 42; g = 50; b = 61; }            // unknown
      else if (v < 50) { r = 27; g = 58; b = 42; }      // free
      else { r = 248; g = 81; b = 73; }                 // occupied
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
      ctx.fillStyle = "#f7c948";
      for (const [wx, wy] of points) {
        const [px, py] = worldToPx(wx, wy);
        ctx.fillRect(px - 1, py - 1, 2, 2);
      }
    }

    // Drone pose
    if (pose) {
      const [px, py] = worldToPx(pose.x, pose.y);
      ctx.fillStyle = "#4cc2ff";
      ctx.beginPath(); ctx.arc(px, py, 5, 0, Math.PI * 2); ctx.fill();
      ctx.strokeStyle = "#4cc2ff"; ctx.lineWidth = 2;
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
