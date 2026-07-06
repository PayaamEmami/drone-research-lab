// Dashboard entry point: wires the websocket stream to the panel renderers.
//
// Frames are keyed by their "type" field:
//   meta   -> { experiment }
//   ranger -> { front, back, left, right, up, down }   (meters or null)
//   state  -> { x, y, z, roll, pitch, yaw }
//   cmd    -> { vx, vy, label }                         (m/s body frame)
//   map    -> { res, width, height, origin:{x,y}, data_b64, pose:{x,y,yaw}, points:[[x,y]] }
import { connect } from "./ws.js";
import { drawHud } from "./hud.js";
import { initChart, pushChart } from "./rangeChart.js";
import { updateState, drawCmd } from "./state.js";
import { drawMap } from "./map.js";

function setMapInfo(text) {
  const el = document.getElementById("map-info");
  if (el) el.textContent = text;
}

function dispatch(frame) {
  const p = frame.payload || {};
  switch (frame.type) {
    case "meta":
      if (p.experiment) {
        document.getElementById("experiment-name").textContent = p.experiment;
        if (/slam|map/i.test(p.experiment)) {
          setMapInfo("waiting for map data...");
        } else {
          setMapInfo("not used in this experiment");
        }
      }
      break;
    case "ranger":
      drawHud(p);
      pushChart(frame.ts, p);
      break;
    case "state":
      updateState(p);
      break;
    case "cmd":
      drawCmd(p);
      break;
    case "map":
      drawMap(p);
      break;
  }
}

initChart();
connect(dispatch);
