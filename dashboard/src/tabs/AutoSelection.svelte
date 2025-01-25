<script lang="ts">
    import { onDestroy } from "svelte";
    import { derived } from "svelte/store";
    import field24 from "../assets/field24.png";
    import { FIELD_HEIGHT, FIELD_WIDTH, ROBOT_SIZE } from "../constants";
    import { AutosActive, AutosOptions, AutosSelected, RobotBlueAlliance } from "../ntStores";

    // import field22 from '../assets/field22.png';
    // import field23 from '../assets/field23.png';
    const field = field24;

    // Option typings.
    type Option = {
        id: string;
        label: string;
        points: Array<[number, number, number, number]>; // x, y, heading, timestamp
        time: number;
    };

    // Parses options from NT value.
    const options = derived(AutosOptions, ($value) => {
        return (
            $value.map((option) => {
                try {
                    const parsed = { ...JSON.parse(option) };
                    if (![`id`, `label`, `points`].every((v) => Object.keys(parsed).includes(v)))
                        throw new Error(`Invalid auto option: ${option}`);
                    return parsed;
                } catch (error) {
                    return null;
                }
            }) ?? []
        ).filter((v) => v !== null) as Option[];
    });

    // Replay positions.
    const createEmptyReplay = (): typeof replay =>
        new Array($options.length).fill(null).map(() => ({ x: Number.MIN_SAFE_INTEGER, y: Number.MIN_SAFE_INTEGER, heading: 0 }));
    let replay: Array<{ x: number; y: number; heading: number }> = createEmptyReplay();
    const timeInterval = setInterval(() => {
        let newReplay: typeof replay = createEmptyReplay();
        const lerp = (s: number, e: number, a: number): number => s + a * (e - s);
        $options.forEach((option, i) => {
            const t = (Date.now() / 1000) % option.time;
            let start = option.points.findLast((point) => point[3] <= t);
            let end = option.points.find((point) => point[3] > t) ?? start;
            if (!start) start = end;
            if (start && end) {
                const a = (t - start![3]) / (end![3] - start![3]);
                const x = lerp(start![0], end![0], a);
                const y = lerp(start![1], end![1], a);
                const heading = lerp(start![2], end![2], a);
                newReplay[i] = {
                    x: $RobotBlueAlliance ? x - ROBOT_SIZE / 2 : FIELD_WIDTH - (x + ROBOT_SIZE / 2),
                    y: FIELD_HEIGHT - y - ROBOT_SIZE / 2,
                    heading: ($RobotBlueAlliance ? Math.PI - heading : heading) * (180 / Math.PI),
                };
            }
        });
        replay = newReplay;
    }, 5);
    onDestroy(() => clearInterval(timeInterval));
</script>

<main>
    <div class="autos-container">
        {#if $options.length}
            {#each $options as { id, label, points }, i}
                <!-- An auto selection. -->
                <button
                    class="auto-selection"
                    class:auto-selection-client="{id === $AutosSelected}"
                    class:auto-selection-robot="{id === $AutosActive}"
                    on:keydown="{() => {}}"
                    on:click="{() => {
                        $AutosSelected = id;
                    }}"
                >
                    <!-- The selection's path. -->
                    <div class="auto-selection-path">
                        <!-- The field. -->
                        <img src="{field}" alt="field" />

                        <!-- The path. -->
                        <svg
                            viewBox="{0} {0} {FIELD_WIDTH} {FIELD_HEIGHT}"
                            fill="none"
                            stroke="var(--auto-line)"
                            stroke-width="0.06"
                            stroke-linecap="round"
                            stroke-linejoin="round"
                        >
                            {#each points as [x, y, _], i}
                                {#if i < points.length - 1}
                                    <line
                                        x1="{$RobotBlueAlliance ? x : FIELD_WIDTH - x}"
                                        y1="{FIELD_HEIGHT - y}"
                                        x2="{$RobotBlueAlliance ? points[i + 1][0] : FIELD_WIDTH - points[i + 1][0]}"
                                        y2="{FIELD_HEIGHT - points[i + 1][1]}"
                                    ></line>
                                {/if}
                            {/each}

                            {#if replay[i] && replay[i].x !== Number.MIN_SAFE_INTEGER}
                                <g transform="rotate({replay[i].heading}, {replay[i].x + ROBOT_SIZE / 2}, {replay[i].y + ROBOT_SIZE / 2})">
                                    <rect x="{replay[i].x}" y="{replay[i].y}" width="{ROBOT_SIZE}" height="{ROBOT_SIZE}"></rect>
                                    <circle cx="{replay[i].x}" cy="{replay[i].y + ROBOT_SIZE / 2}" r="0.1"></circle>
                                </g>
                            {/if}
                        </svg>
                    </div>

                    <!-- The selection's label. -->
                    <p class="auto-selection-label">{label}</p>
                </button>
            {/each}
        {:else}
            <em style="margin-top: 20%; font-size: 3rem;">No available auto options</em>
        {/if}
    </div>
</main>

<style>
    @keyframes selecting {
        0% {
            border-color: var(--accent-color-transparent);
        }
        50% {
            border-color: var(--accent-color);
        }
        100% {
            border-color: var(--accent-color-transparent);
        }
    }

    .autos-container {
        display: flex;
        margin: 2rem;
        justify-content: center;
        flex-wrap: wrap;
        gap: 1.5rem;
    }

    .auto-selection {
        padding: 2rem 2rem 0 2rem;
        background-color: var(--background-tertiary);
        color: inherit;
        font-family: inherit;
        font-size: inherit;
        border: 3px solid transparent;
        border-radius: 1.5rem;
        box-shadow: 0.2rem 0.2rem 0.5rem var(--background-shadow);
        cursor: pointer;
        transition:
            border-color 0.1s,
            transform 0.2s,
            box-shadow 0.2s;
    }

    .auto-selection-client {
        will-change: filter;
        animation-name: selecting;
        animation-duration: 1.5s;
        animation-iteration-count: infinite;
    }

    .auto-selection-robot {
        animation-name: none !important;
        border-color: var(--accent-color);
    }

    .auto-selection:hover,
    .auto-selection-robot {
        box-shadow: 0.3rem 0.3rem 0.5rem var(--background-shadow);
        transform: scale(1.02);
    }

    .auto-selection-label {
        font-weight: 600;
    }

    .auto-selection-path {
        position: relative;
        display: inline-block;
    }

    .auto-selection-path img {
        display: block;
        width: 24rem;
    }

    .auto-selection-path svg {
        position: absolute;
        top: 0;
        left: 0;
    }
</style>
