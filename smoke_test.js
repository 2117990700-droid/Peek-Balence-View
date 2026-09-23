process.env.HTTP_PORT = "18080";
process.env.WS_PORT = "18081";
process.env.FOOT_PORT = "COM999";

const WebSocket = require("ws");
require("./server.js");

const timeout = setTimeout(() => {
  console.error("Smoke test failed: no AI prediction received");
  process.exit(1);
}, 10000);

setTimeout(() => {
  const client = new WebSocket("ws://localhost:18081");
  client.on("open", () => {
    client.send(JSON.stringify({
      type: "foot_features",
      leftPressure: 312,
      rightPressure: 328,
      totalPressure: 640,
      LR: 0.025,
      AP: -0.018,
      sway: 0.055,
      stabilityScore: 92
    }));
    client.send(JSON.stringify({
      type: "nano",
      ai: {
        timestamp: 1,
        chestSway: 0.04,
        waistSway: 0.035,
        swayDifference: 0.005,
        stepFrequency: 1.1,
        balanceScore: 91,
        fallRisk: "LOW"
      }
    }));
  });
  client.on("message", (raw) => {
    const message = JSON.parse(raw.toString());
    if (message.type === "ai_prediction") {
      clearTimeout(timeout);
      console.log(`Smoke test passed: ${message.prediction}, demo=${message.demoMode}`);
      client.close();
      setTimeout(() => process.exit(0), 100);
    }
  });
}, 800);
