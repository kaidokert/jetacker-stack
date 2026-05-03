import { test, expect } from "@playwright/test";

test.describe("Dashboard loads", () => {
  test("index page renders with tabs", async ({ page }) => {
    await page.goto("/");
    await expect(page.locator("h1")).toHaveText("Robot Dashboard");
    await expect(page.locator(".tab-btn")).toHaveCount(2);
    await expect(page.locator(".tab-btn").nth(0)).toHaveText("Overview");
    await expect(page.locator(".tab-btn").nth(1)).toHaveText("MPPI Tuning");
  });

  test("health endpoint returns ok", async ({ request }) => {
    const resp = await request.get("/api/health");
    expect(resp.ok()).toBeTruthy();
    expect(await resp.json()).toEqual({ status: "ok" });
  });
});

test.describe("MPPI Tuning tab", () => {
  test.beforeEach(async ({ page }) => {
    await page.goto("/");
    await page.click(".tab-btn:has-text('MPPI Tuning')");
  });

  test("tab activates and shows MPPI UI", async ({ page }) => {
    await expect(page.locator(".tab-btn.active")).toHaveText("MPPI Tuning");
    await expect(page.locator(".mppi-tuning")).toBeVisible();
  });

  test("status bar controls are present", async ({ page }) => {
    await expect(page.locator(".nav2-badge")).toBeVisible();
    await expect(page.locator("button:has-text('Refresh')")).toBeVisible();
    await expect(page.locator("button:has-text('Reset Position')")).toBeVisible();
    await expect(page.locator(".mppi-clutch")).toBeVisible();
    await expect(page.locator("button:has-text('Retry')")).toBeVisible();
  });

  test("waypoints input has default value", async ({ page }) => {
    const input = page.locator(".mppi-waypoints-input");
    await expect(input).toHaveValue("nav2_matrix_3_forward_left_90");
  });

  test("MPPI Global and Planner panels render with sliders", async ({ page }) => {
    await expect(page.locator("h2:has-text('MPPI Global')")).toBeVisible();
    await expect(page.locator("h2:has-text('Planner')")).toBeVisible();
    const sliders = page.locator(".mppi-global-grid .mppi-slider");
    // 6 MPPI global + 4 planner params
    await expect(sliders).toHaveCount(10);
  });

  test("critic cards are rendered", async ({ page }) => {
    const cards = page.locator(".mppi-critic-card");
    // 9 critics
    await expect(cards).toHaveCount(9);
  });

  test("each critic card has a toggle switch", async ({ page }) => {
    const toggles = page.locator(".mppi-critic-card .mppi-toggle input");
    await expect(toggles).toHaveCount(9);
  });

  test("Live Data panel exists", async ({ page }) => {
    await expect(page.locator("h2:has-text('Live Data')")).toBeVisible();
  });

  test("teleport button calls API and shows result", async ({ page }) => {
    // Intercept the teleport API call
    let apiCalled = false;
    await page.route("/api/mppi/teleport", async (route) => {
      apiCalled = true;
      await route.fulfill({
        status: 200,
        contentType: "application/json",
        body: JSON.stringify({ success: true, message: "Teleported to origin (2.1s)" }),
      });
    });

    await page.click("button:has-text('Reset Position')");
    // Button should show "Teleporting..." while in flight
    // After mock resolves, status message should appear
    await expect(page.locator(".mppi-status-msg")).toHaveText("Teleported to origin (2.1s)");
    expect(apiCalled).toBe(true);
  });

  test("teleport error is displayed", async ({ page }) => {
    await page.route("/api/mppi/teleport", async (route) => {
      await route.fulfill({
        status: 500,
        contentType: "application/json",
        body: JSON.stringify({ success: false, message: "No running stack detected" }),
      });
    });

    await page.click("button:has-text('Reset Position')");
    await expect(page.locator(".mppi-status-msg")).toHaveText(
      "Teleport error: No running stack detected",
    );
  });

  test("clutch toggle is present with checkbox", async ({ page }) => {
    const clutch = page.locator(".mppi-clutch");
    await expect(clutch).toBeVisible();
    const toggle = clutch.locator("input[type=checkbox]");
    await expect(toggle).toHaveCount(1);
    // Clutch label is visible
    await expect(clutch.locator(".mppi-clutch-label")).toHaveText("Clutch");
  });

  test("retry button sends waypoints", async ({ page }) => {
    let sentWaypoints = "";
    await page.route("/api/mppi/retry", async (route, request) => {
      const body = request.postDataJSON();
      sentWaypoints = body.waypoints;
      await route.fulfill({
        status: 200,
        contentType: "application/json",
        body: JSON.stringify({ status: "ok", waypoints: body.waypoints }),
      });
    });

    await page.fill(".mppi-waypoints-input", "nav2_matrix_5_forward_180");
    await page.click("button:has-text('Retry')");
    await expect(page.locator(".mppi-status-msg")).toContainText("nav2_matrix_5_forward_180");
    expect(sentWaypoints).toBe("nav2_matrix_5_forward_180");
  });
});
