import { DEFAULT_URI } from "./constants";
import { NTSvelteClient } from "./lib/NTSvelte";

export const nt = new NTSvelteClient(DEFAULT_URI, { appName: `GRRDashboard` });

export const NTURI = nt.uriReadable();
export const NTConnected = nt.stateReadable();
export const NTBitrate = nt.bitrateReadable();
export const NTLatency = nt.latencyReadable();

export const RobotEnabled = nt.subscribe<boolean>(`/GRRDashboard/Robot/enabled`, false);
export const RobotMatchTime = nt.subscribe<number>(`/GRRDashboard/Robot/matchTime`, 0);
export const RobotBlueAlliance = nt.subscribe<boolean>(`/GRRDashboard/Robot/blueAlliance`, true);
export const RobotVoltage = nt.subscribe<number>(`/GRRDashboard/Robot/voltage`, 0);

export const AutosActive = nt.subscribe<string>(`/GRRDashboard/Autos/active`, ``);
export const AutosOptions = nt.subscribe<string[]>(`/GRRDashboard/Autos/options`, []);
export const AutosSelected = nt.publish<string>(`/GRRDashboard/Autos/selected`, `string`, ``);

nt.connect();
