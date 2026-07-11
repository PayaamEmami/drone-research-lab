// Shared constants for the dashboard UI.

// Beams longer than this (meters) are clamped in the HUD and chart scaling.
export const DISPLAY_RANGE_M = 2.0;

// Horizontal beam directions rendered by the HUD and range chart.
export const DIRS = ["front", "back", "left", "right"];

// Theme palette (VS Code Default Dark Modern — matches tools project).
export const COLORS = {
  accent: "#0078d4",
  accentHover: "#026ec1",
  link: "#4daafc",
  text: "#cccccc",
  muted: "#9d9d9d",
  inactive: "#868686",
  grid: "#2b2b2b",
  gridSubtle: "#3c3c3c",
  bg: "#181818",
  surfaceElevated: "#1f1f1f",
  surfaceOverlay: "#202020",
  surfaceInput: "#313131",
  front: "#4daafc",
  back: "#9e6a03",
  left: "#2ea043",
  right: "#f85149",
  occupied: "#f85149",
  hit: "#9e6a03",
  success: "#2ea043",
  warning: "#9e6a03",
  error: "#f85149",
};
