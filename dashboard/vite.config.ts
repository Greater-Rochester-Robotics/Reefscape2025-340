import { defineConfig } from "vite";
import { svelte } from "@sveltejs/vite-plugin-svelte";

// https://vitejs.dev/config/
export default defineConfig({
    build: {
        minify: !process.env.TAURI_DEBUG ? "esbuild" : false,
        outDir: `./build`,
        sourcemap: !!process.env.TAURI_DEBUG,
        target: process.env.TAURI_PLATFORM == "windows" ? "chrome105" : "safari13",
    },
    clearScreen: false,
    envPrefix: ["VITE_", "TAURI_"],
    plugins: [svelte()],
    server: { strictPort: true },
});
