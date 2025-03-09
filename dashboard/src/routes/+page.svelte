<script lang="ts">
    import { DEFAULT_URI } from "$lib/Constants";
    import { NT } from "$lib/nt/NT.svelte";
    import type { NTData } from "$lib/NTData";

    const nt = new NT(DEFAULT_URI, { appName: `GRRDashboard` });
    nt.subAllNoValues();
    nt.connect();

    let ntData: NTData = $derived.by(() => {
        return {
            uri: nt.getURI(),
            clientState: nt.getState(),
            bitrate: nt.getBitrate(),
            latency: nt.getLatency(),

            enabled: nt.subscribe(`/GRRDashboard/robot/enabled`, false),
            blueAlliance: nt.subscribe(`/GRRDashboard/robot/blueAlliance`, false),
            matchTime: nt.subscribe(`/GRRDashboard/robot/matchTime`, 0),
            voltage: nt.subscribe(`/GRRDashboard/robot/voltage`, 0),

            autoActive: nt.subscribe(`/GRRDashboard/autos/active`, ``),
            autoSelected: nt.publish(`/GRRDashboard/autos/selected`, `string`, ``),
            autoOptions: nt
                .getTopics()
                .entries()
                .filter(([key]) => key.startsWith(`/GRRDashboard/autos/options/`))
                .map(([key]) => ({ name: key, trajectory: nt.subscribe(key, new Uint8Array()) }))
                .toArray(),
        };
    });
</script>

<main>
    <h1>Welcome to Tauri + Svelte</h1>

    <p>{JSON.stringify(ntData)}</p>
</main>

<style>
</style>
