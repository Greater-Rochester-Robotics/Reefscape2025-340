<script lang="ts">
    import { NTBitrate, NTLatency, NTURI, RobotEnabled, RobotMatchTime, RobotVoltage, nt } from "../ntStores";

    import { createEventDispatcher } from "svelte";

    type TabNames = $$Generic<string>;

    /**
     * The tab list.
     * Use on:tabselect to listen for selection changes.
     */
    export let tabs: Array<TabNames>;
    /**
     * The current tab.
     */
    export let currentTab: TabNames;

    // Dispatches the current tab when it changes.
    const dispatch = createEventDispatcher<{ tabselect: TabNames }>();
    $: dispatch(`tabselect`, currentTab);

    // Changes the URI on blur
    const onURIBlur = (e: FocusEvent) => {
        const newURI = decodeURIComponent((e.target as any)?.innerText).trim();
        if (newURI !== $NTURI) nt.setURI(newURI);
    };
</script>

<main>
    <div class="nav-bar">
        <!-- Readouts. -->
        <div class="nav-bar-readouts">
            <!-- Battery Voltage. -->
            <div class="nav-bar-readouts-item">
                <svg viewBox="0 0 14 24" fill="var(--accent-color)" fill-rule="evenodd">
                    <path
                        d="M 4 3 C 4 1.8954 4.8954 1 6 1 L 8 1 C 9.1046 1 10 1.8954 10 3 L 11 3 C 12.1046 3 13 3.8954 13 5 L 13 21 C 13 22.1046 12.1046 23 11 23 L 3 23 C 1.8954 23 1 22.1046 1 21 L 1 5 C 1 3.8954 1.8954 3 3 3 L 4 3 Z M 3 21 L 11 21 L 11 5 L 3 5 L 3 21 Z"
                    ></path>
                </svg>
                <code>{($RobotVoltage ?? 0).toFixed(2)}</code>
            </div>

            <!-- Match Time. -->
            <div class="nav-bar-readouts-item">
                <svg
                    viewBox="0 0 24 24"
                    fill="none"
                    stroke="var(--accent-color)"
                    stroke-width="2"
                    stroke-linecap="round"
                    stroke-linejoin="round"
                >
                    <circle cx="12" cy="12" r="10"></circle>
                    <polyline points="12 6 12 12 16 14"></polyline>
                </svg>
                <code>{new Date(Math.max($RobotMatchTime ?? 0, 0) * 1000).toISOString().substring(15, 19)}</code>
            </div>
        </div>

        <!-- Tab Selection. -->
        <div class="nav-bar-tab-selection">
            <!-- Dynamically creares buttons from the provided tab list. -->
            <!-- When a button is clicked, currentTab is updated, triggering a dispatch from the reactive block under the script tag above. -->
            {#each tabs as tab}
                <button
                    class="nav-bar-tab-selection-button"
                    class:nav-bar-tab-selection-button-current="{tab === currentTab}"
                    on:click="{() => {
                        currentTab = tab;
                    }}">{tab}</button
                >
            {/each}
        </div>

        <!-- Connection Information. -->
        <div class="nav-bar-connection">
            <svg viewBox="0 0 512 512" fill="var(--accent-color)">
                <path d="M443.057,132.943l22.634-22.634a143.764,143.764,0,0,0-211.382,0l22.634,22.634a111.838,111.838,0,0,1,166.114,0Z"
                ></path>
                <path d="M299.615,155.615l22.7,22.7a47.913,47.913,0,0,1,75.362,0l22.7-22.7a79.829,79.829,0,0,0-120.77,0Z"></path>
                <path
                    d="M472,280H376V216H344v64H40a24.028,24.028,0,0,0-24,24V416a24.028,24.028,0,0,0,24,24H472a24.028,24.028,0,0,0,24-24V304A24.028,24.028,0,0,0,472,280Zm-8,128H48V312H464Z"
                ></path>
                <rect width="32" height="32" x="96" y="344"></rect>
                <rect width="32" height="32" x="176" y="344"></rect>
                <rect width="32" height="32" x="256" y="344"></rect>
            </svg>

            <div>
                <!-- Current Mode. -->
                <div style="display: flex; gap: 0.5em; justify-content: center;">
                    <!-- Red circle for disabled, green circle for enabled. -->
                    <svg style="margin: auto 0; height: 0.7em;" viewBox="0 0 100 100" xmlns="http://www.w3.org/2000/svg">
                        <circle fill="{$RobotEnabled ? `var(--good)` : `var(--bad)`}" cx="50" cy="50" r="50"></circle>
                    </svg>
                    <p style="margin: 0;">Robot {$RobotEnabled ? `Enabled` : `Disabled`}</p>
                </div>

                <!-- NT Instance Address. -->
                <code class="nav-bar-connection-address" contenteditable on:blur="{onURIBlur}">{$NTURI}</code>

                <!-- NT Connection Stats. -->
                <code>{($NTBitrate / 1000).toFixed(2)}kb/s | {$NTLatency.toFixed(2)}ms</code>
            </div>
        </div>
    </div>
</main>

<style>
    .nav-bar {
        position: absolute;
        display: flex;
        left: 0;
        bottom: 0;
        margin-left: 1%;
        margin-bottom: 0.75%;
        height: 9.5%;
        width: 98%;
        background-color: var(--background-secondary);
        border-radius: 1rem;
        box-shadow: 0.2rem 0.2rem 1rem var(--background-shadow);
        justify-content: space-between;
    }

    .nav-bar-readouts {
        display: flex;
        margin: 0.5rem 2rem;
        width: 20vw;
        gap: 2rem;
        justify-content: left;
    }

    .nav-bar-readouts-item {
        display: flex;
        gap: 0.8rem;
    }

    .nav-bar-readouts-item > svg {
        margin: auto;
        height: 2.3rem;
    }

    .nav-bar-readouts-item > code {
        margin: auto;
        font-size: 2rem;
        font-weight: 600;
    }

    .nav-bar-tab-selection {
        display: flex;
        margin: 1rem 2rem;
        align-self: center;
        gap: 0.8rem;
    }

    .nav-bar-tab-selection-button {
        padding: 0.4rem 1.2rem;
        font-size: 1rem;
        font-weight: 600;
        font-family: inherit;
        color: var(--text-primary);
        background-color: var(--background-tertiary);
        border-radius: 8px;
        border: 3px solid transparent;
        cursor: pointer;
        white-space: nowrap;
        box-shadow: 0.2rem 0.2rem 0.5rem var(--background-shadow);
        transition:
            border-color 0.4s,
            transform 0.2s,
            box-shadow 0.2s;
    }

    .nav-bar-tab-selection-button:hover,
    .nav-bar-tab-selection-button-current {
        box-shadow: 0.3rem 0.3rem 0.5rem var(--background-shadow);
        transform: scale(1.05);
    }

    .nav-bar-tab-selection-button-current {
        border-color: var(--accent-color);
    }

    .nav-bar-connection {
        display: flex;
        margin: 0rem 2rem;
        width: 20vw;
        order: 3;
        gap: 1.2rem;
        justify-content: right;
    }

    .nav-bar-connection > svg {
        margin: auto 0;
        height: 2.8rem;
    }

    .nav-bar-connection > div {
        display: flex;
        gap: 0.2rem;
        min-width: 22ch;
        text-align: center;
        font-size: 0.7rem;
        flex-direction: column;
        justify-content: center;
        font-weight: 600;
    }

    .nav-bar-connection > div > * {
        margin: 0;
    }

    .nav-bar-connection-address {
        padding: 0.1rem;
        border-radius: 0.2rem;
        background-color: var(--background-tertiary);
        outline: 0 solid transparent;
    }

    .nav-bar-connection-address::selection {
        background-color: var(--accent-color);
    }

    .nav-bar-connection-address:focus {
        outline: 2px solid var(--accent-color);
    }
</style>
