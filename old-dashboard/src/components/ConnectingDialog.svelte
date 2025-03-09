<script lang="ts">
    import { NTURI, nt } from "../ntStores";

    const onURIBlur = (e: FocusEvent) => {
        const newURI = decodeURIComponent((e.target as any)?.innerText).trim();
        if (newURI !== $NTURI) nt.setURI(newURI);
    };
</script>

<main>
    <svg class="wifi" viewBox="0 0 24 24" fill="var(--accent-color)">
        <path
            d="M1 9l2 2c4.97-4.97 13.03-4.97 18 0l2-2C16.93 2.93 7.08 2.93 1 9zm8 8l3 3 3-3c-1.65-1.66-4.34-1.66-6 0zm-4-4l2 2c2.76-2.76 7.24-2.76 10 0l2-2C15.14 9.14 8.87 9.14 5 13z"
        ></path>
    </svg>

    <h1>Connecting...</h1>
    <code class="address" contenteditable on:blur="{onURIBlur}">{$NTURI}</code>
</main>

<style>
    @keyframes glow {
        0% {
            filter: drop-shadow(0 0 3rem var(--accent-color-transparent));
        }
        50% {
            filter: drop-shadow(0 0 3rem var(--accent-color));
        }
        100% {
            filter: drop-shadow(0 0 3rem var(--accent-color-transparent));
        }
    }

    .wifi {
        height: 8rem;
        will-change: filter;
        animation-name: glow;
        animation-duration: 4s;
        animation-iteration-count: infinite;
    }

    .address {
        font-size: 1.2rem;
        padding: 0.5rem;
        border-radius: 0.5rem;
        background-color: var(--background-secondary);
        outline: 0 solid transparent;
    }

    .address::selection {
        background-color: var(--accent-color);
    }

    .address:focus {
        outline: 2px solid var(--accent-color);
    }
</style>
