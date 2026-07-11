// Unit-specific raw vs. filtered traces for the state-estimation experiment.
// Frame: estimate -> { <channel>: { raw, filtered } }
import { COLORS } from "./constants.js";

const MAX_POINTS = 150;
const MIN_METRIC_POINTS = 12;
let t0 = null;

const CHANNELS = {
  front: { color: COLORS.front, unit: "m" },
  back: { color: COLORS.back, unit: "m" },
  left: { color: COLORS.left, unit: "m" },
  right: { color: COLORS.right, unit: "m" },
  height: { color: COLORS.accent, unit: "m" },
  roll: { color: COLORS.success, unit: "°" },
  pitch: { color: COLORS.link, unit: "°" },
  yaw: { color: COLORS.warning, unit: "°" },
};

const GROUPS = [
  {
    id: "estimate-range-chart",
    channels: ["front", "back", "left", "right"],
    axisTitle: "range (m)",
  },
  {
    id: "estimate-height-chart",
    channels: ["height"],
    axisTitle: "height (m)",
  },
  {
    id: "estimate-orientation-chart",
    channels: ["roll", "pitch", "yaw"],
    axisTitle: "angle (deg)",
  },
];

const charts = new Map();
const history = new Map();

function dataset(channel, kind) {
  const raw = kind === "raw";
  const color = CHANNELS[channel].color;
  return {
    label: `${channel} ${kind}`,
    channel,
    kind,
    borderColor: color,
    backgroundColor: color,
    data: [],
    pointRadius: 0,
    borderWidth: raw ? 1 : 2,
    borderDash: raw ? [4, 3] : [],
    tension: 0.2,
    spanGaps: false,
  };
}

function createChart(group) {
  const el = document.getElementById(group.id);
  if (!el) return null;
  const datasets = group.channels.flatMap((channel) => [
    dataset(channel, "raw"),
    dataset(channel, "filtered"),
  ]);
  return new Chart(el.getContext("2d"), {
    type: "line",
    data: { labels: [], datasets },
    options: {
      animation: false,
      responsive: true,
      maintainAspectRatio: false,
      interaction: { mode: "index", intersect: false },
      scales: {
        x: {
          title: { display: true, text: "elapsed time (s)", color: COLORS.muted },
          ticks: { color: COLORS.muted, maxTicksLimit: 6 },
          grid: { color: COLORS.grid },
        },
        y: {
          title: { display: true, text: group.axisTitle, color: COLORS.muted },
          ticks: { color: COLORS.muted },
          grid: { color: COLORS.grid },
        },
      },
      plugins: {
        legend: {
          labels: {
            color: COLORS.text,
            boxWidth: 12,
            usePointStyle: true,
            pointStyle: "line",
          },
        },
      },
    },
  });
}

function ensureCharts() {
  for (const group of GROUPS) {
    if (!charts.has(group.id)) {
      const chart = createChart(group);
      if (chart) charts.set(group.id, chart);
    }
  }
  return charts.size === GROUPS.length;
}

function numeric(value) {
  return value != null && Number.isFinite(value) ? value : null;
}

function appendHistory(channel, raw, filtered) {
  if (!history.has(channel)) history.set(channel, { raw: [], filtered: [] });
  const values = history.get(channel);
  values.raw.push(raw);
  values.filtered.push(filtered);
  if (values.raw.length > MAX_POINTS) values.raw.shift();
  if (values.filtered.length > MAX_POINTS) values.filtered.shift();
}

function standardDeviation(values) {
  const valid = values.filter((value) => value != null);
  if (valid.length < MIN_METRIC_POINTS) return null;
  const mean = valid.reduce((sum, value) => sum + value, 0) / valid.length;
  const variance = valid.reduce((sum, value) => sum + (value - mean) ** 2, 0) / valid.length;
  return Math.sqrt(variance);
}

function formatValue(value, unit) {
  if (value == null) return "—";
  const precision = unit === "m" ? 3 : 2;
  return `${value.toFixed(precision)} ${unit}`;
}

function addCell(row, text, className = "") {
  const cell = document.createElement("td");
  cell.textContent = text;
  if (className) cell.className = className;
  row.appendChild(cell);
}

function updateMetrics(payload) {
  const body = document.getElementById("estimate-metrics");
  if (!body) return;
  body.replaceChildren();

  for (const channel of Object.keys(CHANNELS)) {
    const values = history.get(channel);
    const current = payload[channel];
    if (!values || !current) continue;

    const rawStd = standardDeviation(values.raw);
    const filteredStd = standardDeviation(values.filtered);
    const reduction = rawStd != null && rawStd > 1e-9 && filteredStd != null
      ? 100 * (1 - filteredStd / rawStd)
      : null;
    const raw = numeric(current.raw);
    const filtered = numeric(current.filtered);
    const residual = raw != null && filtered != null ? raw - filtered : null;
    const unit = CHANNELS[channel].unit;

    const row = document.createElement("tr");
    addCell(row, channel);
    addCell(row, formatValue(rawStd, unit));
    addCell(row, formatValue(filteredStd, unit));
    addCell(
      row,
      reduction == null ? "collecting…" : `${reduction.toFixed(0)}%`,
      reduction == null ? "" : reduction >= 0 ? "metric-good" : "metric-warn",
    );
    addCell(row, formatValue(residual, unit));
    body.appendChild(row);
  }
}

export function updateEstimate(payload) {
  if (!ensureCharts()) return;
  if (t0 == null) t0 = performance.now();
  const elapsed = ((performance.now() - t0) / 1000).toFixed(1);

  for (const [channel, value] of Object.entries(payload)) {
    if (!(channel in CHANNELS)) continue;
    appendHistory(channel, numeric(value?.raw), numeric(value?.filtered));
  }

  for (const group of GROUPS) {
    const chart = charts.get(group.id);
    chart.data.labels.push(elapsed);
    if (chart.data.labels.length > MAX_POINTS) chart.data.labels.shift();

    for (const series of chart.data.datasets) {
      const value = payload[series.channel];
      series.data.push(numeric(value?.[series.kind]));
      if (series.data.length > MAX_POINTS) series.data.shift();
    }
    chart.update("none");
  }

  updateMetrics(payload);
}
