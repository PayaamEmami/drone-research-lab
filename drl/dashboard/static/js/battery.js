// Battery readout in the top bar (always visible across experiments).
// Frame: battery -> { vbat: float }  (volts, from pm.vbat)

function packScale(vbat) {
  // >6 V is treated as 2S; otherwise 1S (typical Crazyflie packs).
  return vbat > 6 ? 2 : 1;
}

function estimatePercent(vbat) {
  if (vbat == null || !Number.isFinite(vbat)) return null;
  const scale = packScale(vbat);
  const vmin = 3.0 * scale;
  const vmax = 4.2 * scale;
  const pct = ((vbat - vmin) / (vmax - vmin)) * 100;
  return Math.max(0, Math.min(100, Math.round(pct)));
}

function levelClass(vbat) {
  if (vbat == null || !Number.isFinite(vbat)) return "";
  const scale = packScale(vbat);
  if (vbat < 3.3 * scale) return "bad";
  if (vbat < 3.5 * scale) return "warn";
  return "good";
}

export function updateBattery(p) {
  const el = document.getElementById("battery-readout");
  if (!el) return;
  const v = p.vbat;
  el.classList.remove("good", "warn", "bad");
  if (v == null || !Number.isFinite(v)) {
    el.textContent = "bat: --";
    return;
  }
  const pct = estimatePercent(v);
  el.textContent = `bat: ${v.toFixed(2)} V (~${pct}%)`;
  const cls = levelClass(v);
  if (cls) el.classList.add(cls);
}
