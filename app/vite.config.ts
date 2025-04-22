/// <reference types="vitest" />
/// <reference types="vite/client" />

import yaml from "@modyfi/vite-plugin-yaml";
import react from "@vitejs/plugin-react";
import { defineConfig } from "vite";

// https://vite.dev/config/
export default defineConfig({
  plugins: [react(), yaml()],
  test: {
    globals: true,
    environment: "jsdom",
    setupFiles: "./src/setupTests.ts",
  },
});
