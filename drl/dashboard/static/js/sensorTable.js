// Raw-values table for the six range beams: distance in meters and millimeters
// plus an in-range/out-of-range status, updated from each `ranger` frame.
import { proximityColor } from "./hud.js";

const BEAMS = ["front", "back", "left", "right", "up", "down"];

let rows = null;

function buildRows() {
  const body = document.getElementById("sensor-table-body");
  if (!body) return null;
  const map = {};
  for (const beam of BEAMS) {
    const tr = document.createElement("tr");
    tr.innerHTML =
      `<td>${beam}</td>` +
      `<td data-k="m">--</td>` +
      `<td data-k="mm">--</td>` +
      `<td data-k="status"><span class="dot"></span> --</td>`;
    body.appendChild(tr);
    map[beam] = tr;
  }
  return map;
}

export function updateSensorTable(p) {
  if (!rows) rows = buildRows();
  if (!rows) return;

  for (const beam of BEAMS) {
    const tr = rows[beam];
    const d = p[beam];
    const inRange = d != null;
    tr.querySelector('[data-k="m"]').textContent = inRange ? d.toFixed(2) : "--";
    tr.querySelector('[data-k="mm"]').textContent = inRange ? String(Math.round(d * 1000)) : "--";
    const status = tr.querySelector('[data-k="status"]');
    const dot = status.querySelector(".dot");
    dot.style.background = proximityColor(d);
    status.lastChild.textContent = inRange ? " in range" : " out of range";
  }
}
