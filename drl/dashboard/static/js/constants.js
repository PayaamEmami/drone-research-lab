// Shared constants for the dashboard UI.

// Beams longer than this (meters) are clamped in the HUD and chart scaling.
export const DISPLAY_RANGE_M = 2.0;

// Horizontal beam directions rendered by the HUD and range chart.
export const DIRS = ["front", "back", "left", "right"];

// Theme palette (kept in sync with styles.css).
export const COLORS = {
  accent: "#4cc2ff",
  text: "#e6edf3",
  muted: "#8b97a7",
  grid: "#2a323d",
  bg: "#0e1116",
  front: "#4cc2ff",
  back: "#d29922",
  left: "#3fb950",
  right: "#f778ba",
  occupied: "#f85149",
  hit: "#f7c948",
};
