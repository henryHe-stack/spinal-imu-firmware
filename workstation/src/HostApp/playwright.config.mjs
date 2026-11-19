import { defineConfig } from "@playwright/test";

export default defineConfig({
  testDir: "./tests",
  testMatch: "interaction.spec.mjs",
  workers: 1,
  reporter: "line",
  use: {
    channel: "msedge",
    viewport: { width: 1440, height: 1000 },
  },
  webServer: {
    command: "node server.mjs",
    url: "http://127.0.0.1:4173",
    reuseExistingServer: true,
  },
});
