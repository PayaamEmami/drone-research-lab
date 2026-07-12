// Dashboard entry point: wires the websocket stream to the panel renderers.
//
// Frames are keyed by their "type" field:
//   meta   -> { experiment }
//   ranger -> { front, back, left, right, up, down }   (meters or null)
//   state  -> { x, y, z, roll, pitch, yaw }
//   cmd    -> { vx, vy, label }                         (m/s body frame)
//   map    -> { res, width, height, origin:{x,y}, data_b64, pose, pose_raw, trail, points }
//   estimate -> { <channel>: { raw, filtered } }
//   traj   -> { reference:{x,y,z}, estimate:{x,y,z}, command:{vx,vy,vz} }
//   cloud  -> { points: [[x,y,z], ...] }
//   battery -> { vbat }                               (volts)
import { connect } from "./ws.js";
import { drawHud } from "./hud.js";
import { initChart, pushChart } from "./rangeChart.js";
import { updateSensorTable } from "./sensorTable.js";
import { updateState, drawCmd } from "./state.js";
import { drawMap } from "./map.js";
import { updateEstimate } from "./estimate.js";
import { drawTraj } from "./traj.js";
import { drawCloud } from "./cloud.js";
import { updateBattery } from "./battery.js";

function setMapInfo(text) {
  const el = document.getElementById("map-info");
  if (el) el.textContent = text;
}

function setCardVisible(id, visible) {
  const el = document.getElementById(id);
  if (el) el.classList.toggle("hidden", !visible);
}

/** Show only the panels relevant to the active experiment. */
function configurePanels(experiment) {
  const name = (experiment || "").toLowerCase();
  const isStateEst = /state estimation/.test(name);
  const isTraj = /trajectory/.test(name);
  const isSlam = /slam/.test(name);
  const isProximity = /proximity/.test(name);

  setCardVisible("card-map", isSlam);
  setCardVisible("card-cloud", isSlam);
  setCardVisible("card-estimate", isStateEst);
  setCardVisible("card-traj", isTraj);
  setCardVisible("card-sensor-table", isProximity);

  // Proximity HUD is useful for any live sensing experiment.
  const sensing = isStateEst || isSlam || isTraj || isProximity;
  setCardVisible("card-hud", sensing);
  // Proximity sensing keeps a minimal layout: HUD + raw-values table only.
  setCardVisible("card-chart", sensing && !isProximity);
  setCardVisible("card-state", sensing && !isProximity);

  // State estimation only observes sensors; it sends no flight commands.
  setCardVisible("command-panel", !isStateEst && (isSlam || isTraj));
  const stateTitle = document.getElementById("state-card-title");
  if (stateTitle) {
    stateTitle.textContent = isStateEst ? "Onboard state estimate" : "State estimate";
  }
}

function dispatch(frame) {
  const p = frame.payload || {};
  switch (frame.type) {
    case "meta":
      if (p.experiment) {
        document.getElementById("experiment-name").textContent = p.experiment;
        configurePanels(p.experiment);
        if (/slam/i.test(p.experiment)) {
          setMapInfo("waiting for map data...");
        }
      }
      break;
    case "ranger":
      drawHud(p);
      pushChart(frame.ts, p);
      updateSensorTable(p);
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
    case "estimate":
      updateEstimate(p);
      break;
    case "traj":
      drawTraj(p);
      break;
    case "cloud":
      drawCloud(p);
      break;
    case "battery":
      updateBattery(p);
      break;
  }
}

initChart();
connect(dispatch);
