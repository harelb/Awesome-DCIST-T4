import {StrictMode} from "react";
import {createRoot} from "react-dom/client";
import {App} from "./App";
import {initToken} from "./lib/auth";
import {ws} from "./ws";
import {bindFleet, useFleet} from "./stores/fleet";
import {bindJobs, useJobs} from "./stores/jobs";
import {bindRobots} from "./stores/robots";
import {bindRobotColors} from "./stores/robotColors";
import {bindTheme} from "./stores/theme";
import {bindExperiment} from "./stores/experiment";
import "./styles.css";
import "./theme.css";

bindTheme();
initToken();
bindFleet();
bindJobs();
bindRobots();
bindRobotColors();
bindExperiment();
ws.start();
void useFleet.getState().load();
void useJobs.getState().load();
// health is cheap: keep the top-bar indicators fresh even without WS traffic
window.setInterval(() => void useFleet.getState().load(), 15_000);

createRoot(document.getElementById("root")!).render(
  <StrictMode>
    <App />
  </StrictMode>,
);
