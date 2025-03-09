<script lang="ts">
    const IGNORE_CONNECTION = false;

    import ConnectingDialog from "./components/ConnectingDialog.svelte";
    import NavBar from "./components/NavBar.svelte";

    import AutoSelection from "./tabs/AutoSelection.svelte";
    import DriverView from "./tabs/DriverView.svelte";

    import { NTConnected } from "./ntStores";
    import { NTSvelteClientState } from "./lib/NTSvelte";

    // Define your tabs here.
    // The navbar will be automatically populated using this object, and tabs will be dynamically displayed when selected.
    // The key should be the name of the tab as it appears on the nav bar.
    // The value should be the tab's svelte component.
    const tabs = {
        "Driver View": DriverView,
        "Auto Selection": AutoSelection,
    };

    // Helpers for tab selection.
    const tabNames: Array<keyof typeof tabs> = Object.keys(tabs) as any;
    const storedTab = tabNames.includes(localStorage.currentTab) ? (localStorage.currentTab as keyof typeof tabs) : tabNames[0];
    let currentTab = storedTab;
    $: localStorage.currentTab = currentTab;
</script>

<main>
    <!-- If connected, show the current tab and the navbar. -->
    {#if $NTConnected === NTSvelteClientState.CONNECTED || IGNORE_CONNECTION}
        <!-- The selected tab. -->
        <div class="tab">
            <svelte:component this="{tabs[currentTab]}" />
        </div>

        <!-- The navbar. -->
        <NavBar
            tabs="{tabNames}"
            {currentTab}
            on:tabselect="{({ detail }) => {
                currentTab = detail;
            }}"
        />
    {:else}
        <!-- If not connected, show the connecting dialog. -->
        <ConnectingDialog />
    {/if}
</main>

<style>
    .tab {
        position: absolute;
        left: 0;
        top: 0;
        margin: 1% 1%;
        height: 84%;
        width: 98%;
        background-color: var(--background-secondary);
        border-radius: 1rem;
        box-shadow: 0.2rem 0.2rem 1rem var(--background-shadow);
        overflow: auto;
    }

    .tab::-webkit-scrollbar {
        display: none;
    }
</style>
