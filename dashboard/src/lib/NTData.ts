import type { NTClientState } from "./nt/NT.svelte";

export interface NTData {
    uri: string;
    clientState: NTClientState;
    bitrate: number;
    latency: number;

    enabled: boolean;
    blueAlliance: boolean;
    matchTime: number;
    voltage: number;

    autoActive: string;
    autoSelected: string | null;
    autoOptions: Array<{
        name: string;
        trajectory: Uint8Array;
    }>;
}
