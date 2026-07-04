// Rolling time-series of the four horizontal beams, backed by Chart.js
// (loaded globally via CDN as `window.Chart`).
import { DISPLAY_RANGE_M, COLORS } from "./constants.js";

const MAX_POINTS = 150;

let rangeChart = null;
let chartT0 = null;

export function initChart() {
  const ctx = document.getElementById("range-chart").getContext("2d");
  const mkSet = (label, color) => ({
    label, borderColor: color, backgroundColor: color,
    data: [], pointRadius: 0, borderWidth: 2, tension: 0.2, spanGaps: false,
  });
  rangeChart = new Chart(ctx, {
    type: "line",
    data: {
      labels: [],
      datasets: [
        mkSet("front", COLORS.front),
        mkSet("back", COLORS.back),
        mkSet("left", COLORS.left),
        mkSet("right", COLORS.right),
      ],
    },
    options: {
      animation: false,
      responsive: true,
      scales: {
        x: { ticks: { color: COLORS.muted, maxTicksLimit: 6 }, grid: { color: COLORS.grid } },
        y: { min: 0, suggestedMax: DISPLAY_RANGE_M, title: { display: true, text: "meters", color: COLORS.muted },
             ticks: { color: COLORS.muted }, grid: { color: COLORS.grid } },
      },
      plugins: { legend: { labels: { color: COLORS.text, boxWidth: 12 } } },
    },
  });
}

export function pushChart(ts, p) {
  if (!rangeChart) return;
  if (chartT0 == null) chartT0 = ts;
  const label = (ts - chartT0).toFixed(1);
  rangeChart.data.labels.push(label);
  rangeChart.data.datasets[0].data.push(p.front);
  rangeChart.data.datasets[1].data.push(p.back);
  rangeChart.data.datasets[2].data.push(p.left);
  rangeChart.data.datasets[3].data.push(p.right);
  if (rangeChart.data.labels.length > MAX_POINTS) {
    rangeChart.data.labels.shift();
    rangeChart.data.datasets.forEach((d) => d.data.shift());
  }
  rangeChart.update();
}
