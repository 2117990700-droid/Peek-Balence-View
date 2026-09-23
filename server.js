const fs = require("fs");
const path = require("path");
const http = require("http");
const readline = require("readline");
const { spawn } = require("child_process");
const WebSocket = require("ws");
const { SerialPort, ReadlineParser } = require("serialport");

const HTTP_PORT = Number(process.env.HTTP_PORT || 8080);
const WS_PORT = Number(process.env.WS_PORT || 8081);
const FOOT_PORT = process.env.FOOT_PORT || "COM3";
const FOOT_BAUD_RATE = 115200;
const PYTHON_BIN = process.env.PYTHON_BIN || findPython();
const PYTHON_ENV = {
  ...process.env,
  PYTHONUTF8: "1",
  PYTHONIOENCODING: "utf-8"
};
const EMIT_INTERVAL_MS = 500;
const FOOT_MAX_AGE_MS = 15000;

const ROOT_DIR = __dirname;
const DATA_DIR = path.join(ROOT_DIR, "data");
const DATASET_PATH = path.join(DATA_DIR, "balance_dataset.csv");
const DASHBOARD_PATH = path.join(ROOT_DIR, "finalUI.html");
const TRAIN_SCRIPT = path.join(ROOT_DIR, "ai_prediction", "train_model.py");
const PREDICT_SCRIPT = path.join(ROOT_DIR, "ai_prediction", "predict_stream.py");

const FEATURES = [
  "chestSway",
  "waistSway",
  "swayDifference",
  "stepFrequency",
  "imuBalanceScore",
  "leftPressure",
  "rightPressure",
  "totalPressure",
  "lrBalance",
  "apBalance",
  "footSway",
  "footStabilityScore"
];

const VALID_LABELS = new Set(["NORMAL", "WARNING", "HIGH_RISK"]);

let imuData = null;
let footData = null;
let currentLabel = null;
let collecting = false;
let modelReady = false;
let modelDemoMode = false;
let modelMessage = "模型未训练";
let training = false;
let lastEmitAt = 0;
let sampleCount = 0;
let predictor = null;
let predictionSequence = 0;

fs.mkdirSync(DATA_DIR, { recursive: true });
ensureDataset();
sampleCount = countSamples();

// Serve the dashboard without requiring VS Code Live Server.
const httpServer = http.createServer((request, response) => {
  if (request.url === "/health") {
    response.writeHead(200, { "Content-Type": "application/json; charset=utf-8" });
    response.end(JSON.stringify(buildStatus()));
    return;
  }

  if (request.url !== "/" && request.url !== "/finalUI.html") {
    response.writeHead(404, { "Content-Type": "text/plain; charset=utf-8" });
    response.end("Not found");
    return;
  }

  fs.readFile(DASHBOARD_PATH, (error, content) => {
    if (error) {
      response.writeHead(500, { "Content-Type": "text/plain; charset=utf-8" });
      response.end(`Cannot read finalUI.html: ${error.message}`);
      return;
    }
    response.writeHead(200, { "Content-Type": "text/html; charset=utf-8" });
    response.end(content);
  });
});

httpServer.on("error", (error) => handleListenError("HTTP", HTTP_PORT, error));
httpServer.listen(HTTP_PORT, () => {
  console.log(`Dashboard: http://localhost:${HTTP_PORT}`);
});

const wss = new WebSocket.Server({ port: WS_PORT });
wss.on("error", (error) => handleListenError("WebSocket", WS_PORT, error));
wss.on("listening", () => {
  console.log(`WebSocket server: ws://localhost:${WS_PORT}`);
});

wss.on("connection", (socket) => {
  console.log("WebSocket client connected");
  send(socket, { type: "system_status", ...buildStatus() });

  socket.on("message", (message) => {
    const text = message.toString().trim();
    if (text.startsWith("ROLE:")) return;

    let data;
    try {
      data = JSON.parse(text);
    } catch {
      console.log("Ignored non-JSON WebSocket message:", text);
      return;
    }

    if (handleControlMessage(data, socket)) return;
    processSensorData(data, socket);
  });

  socket.on("close", () => console.log("WebSocket client disconnected"));
  socket.on("error", (error) => console.log("WebSocket error:", error.message));
});

function startFootSerial() {
  let serial;
  try {
    serial = new SerialPort({
      path: FOOT_PORT,
      baudRate: FOOT_BAUD_RATE,
      autoOpen: false
    });
  } catch (error) {
    console.log("Cannot create foot serial connection:", error.message);
    return;
  }

  const parser = serial.pipe(new ReadlineParser({ delimiter: "\n" }));

  serial.open((error) => {
    if (error) {
      console.log(`Cannot open ${FOOT_PORT}: ${error.message}`);
      console.log("Set the correct port with: set FOOT_PORT=COMx");
      return;
    }
    console.log(`Foot sensor connected on ${FOOT_PORT}`);
  });

  serial.on("error", (error) => console.log("Foot serial error:", error.message));
  parser.on("data", (line) => {
    const text = line.trim();
    if (!text.startsWith("{")) return;
    try {
      processSensorData(JSON.parse(text));
    } catch {
      console.log("Invalid foot JSON:", text);
    }
  });
}

function processSensorData(data, sender = null) {
  if (data.type === "nano" && data.ai) {
    imuData = {
      receivedAt: Date.now(),
      timestamp: data.ai.timestamp,
      chestSway: toNumber(data.ai.chestSway),
      waistSway: toNumber(data.ai.waistSway),
      swayDifference: toNumber(data.ai.swayDifference),
      stepFrequency: toNumber(data.ai.stepFrequency),
      balanceScore: toNumber(data.ai.balanceScore),
      fallRisk: String(data.ai.fallRisk || data.fallRisk || "UNKNOWN")
    };
  }

  if (data.type === "foot_features") {
    footData = {
      receivedAt: Date.now(),
      timestamp: data.timestamp ?? data.time ?? null,
      leftPressure: toNumber(data.leftPressure),
      rightPressure: toNumber(data.rightPressure),
      totalPressure: toNumber(data.totalPressure),
      lrBalance: toNumber(data.lrBalance ?? data.LR),
      apBalance: toNumber(data.apBalance ?? data.AP),
      footSway: toNumber(data.sway),
      footStabilityScore: toNumber(data.stabilityScore)
    };
  }

  broadcast(data, sender);

  const now = Date.now();
  if (now - lastEmitAt < EMIT_INTERVAL_MS) return;
  const fused = getFusedFeatures(now);
  if (!fused) return;

  lastEmitAt = now;
  broadcast({ type: "ai_features", data: fused });
  saveSample(fused);
  requestPrediction(fused);
}

function getFusedFeatures(now = Date.now()) {
  if (!imuData || !footData) return null;
  if (now - footData.receivedAt > FOOT_MAX_AGE_MS) return null;

  const fused = {
    timestamp: now,
    chestSway: imuData.chestSway,
    waistSway: imuData.waistSway,
    swayDifference: imuData.swayDifference,
    stepFrequency: imuData.stepFrequency,
    imuBalanceScore: imuData.balanceScore,
    leftPressure: footData.leftPressure,
    rightPressure: footData.rightPressure,
    totalPressure: footData.totalPressure,
    lrBalance: footData.lrBalance,
    apBalance: footData.apBalance,
    footSway: footData.footSway,
    footStabilityScore: footData.footStabilityScore,
    ruleBasedRisk: imuData.fallRisk
  };

  return FEATURES.every((name) => Number.isFinite(fused[name])) ? fused : null;
}

function handleControlMessage(data, socket) {
  if (data.type === "set_label") {
    const label = String(data.label || "").toUpperCase();
    if (!VALID_LABELS.has(label)) {
      send(socket, { type: "control_error", message: "无效标签" });
      return true;
    }
    currentLabel = label;
    broadcastStatus();
    console.log(`Current label: ${currentLabel}`);
    return true;
  }

  if (data.type === "collection") {
    if (data.enabled && !currentLabel) {
      send(socket, { type: "control_error", message: "请先选择数据标签" });
      return true;
    }
    collecting = Boolean(data.enabled);
    broadcastStatus();
    console.log(`Data collection: ${collecting ? "ON" : "OFF"}`);
    return true;
  }

  if (data.type === "train_model") {
    trainModel();
    return true;
  }

  if (data.type === "get_status") {
    send(socket, { type: "system_status", ...buildStatus() });
    return true;
  }

  return false;
}

function ensureDataset() {
  if (fs.existsSync(DATASET_PATH)) return;
  const header = ["timestamp", ...FEATURES, "label"].join(",") + "\n";
  fs.writeFileSync(DATASET_PATH, header, "utf8");
}

function countSamples() {
  try {
    const lines = fs.readFileSync(DATASET_PATH, "utf8").trim().split(/\r?\n/);
    return Math.max(0, lines.length - 1);
  } catch {
    return 0;
  }
}

function saveSample(fused) {
  if (!collecting || !currentLabel) return;
  const values = [fused.timestamp, ...FEATURES.map((name) => fused[name]), currentLabel];
  fs.appendFileSync(DATASET_PATH, values.join(",") + "\n", "utf8");
  sampleCount += 1;
  broadcastStatus();
}

function startPredictor() {
  predictor = spawn(PYTHON_BIN, ["-u", PREDICT_SCRIPT], {
    cwd: ROOT_DIR,
    windowsHide: true,
    env: PYTHON_ENV,
    stdio: ["pipe", "pipe", "pipe"]
  });

  const output = readline.createInterface({ input: predictor.stdout });
  output.on("line", (line) => {
    let message;
    try {
      message = JSON.parse(line);
    } catch {
      console.log("Predictor output:", line);
      return;
    }

    if (message.type === "model_status") {
      modelReady = Boolean(message.modelReady);
      modelDemoMode = Boolean(message.demoMode);
      modelMessage = message.message || (modelReady ? "模型已加载" : "模型未训练");
      broadcastStatus();
      return;
    }

    if (message.type === "ai_prediction") {
      modelReady = true;
      modelDemoMode = Boolean(message.demoMode);
      modelMessage = modelDemoMode ? "演示模型（合成样本）" : "模型已加载";
      broadcast(message);
    }
  });

  predictor.stderr.on("data", (chunk) => {
    const message = chunk.toString().trim();
    if (message) console.log("Python:", message);
  });

  predictor.on("error", (error) => {
    modelReady = false;
    modelMessage = `无法启动 Python: ${error.message}`;
    broadcastStatus();
  });

  predictor.on("exit", (code) => {
    modelReady = false;
    modelMessage = `预测进程已停止 (${code})`;
    predictor = null;
    broadcastStatus();
  });
}

function requestPrediction(fused) {
  if (!predictor || !predictor.stdin.writable) return;
  predictionSequence += 1;
  predictor.stdin.write(JSON.stringify({
    id: predictionSequence,
    timestamp: fused.timestamp,
    features: Object.fromEntries(FEATURES.map((name) => [name, fused[name]]))
  }) + "\n");
}

function trainModel() {
  if (training) return;
  training = true;
  broadcast({ type: "training_status", running: true, message: "模型训练中…" });

  const trainer = spawn(PYTHON_BIN, [TRAIN_SCRIPT], {
    cwd: ROOT_DIR,
    windowsHide: true,
    env: PYTHON_ENV,
    stdio: ["ignore", "pipe", "pipe"]
  });

  let stdout = "";
  let stderr = "";
  trainer.stdout.on("data", (chunk) => { stdout += chunk.toString(); });
  trainer.stderr.on("data", (chunk) => { stderr += chunk.toString(); });

  trainer.on("error", (error) => {
    training = false;
    broadcast({ type: "training_status", running: false, success: false, message: error.message });
  });

  trainer.on("exit", (code) => {
    training = false;
    const message = (code === 0 ? stdout : stderr || stdout).trim();
    broadcast({
      type: "training_status",
      running: false,
      success: code === 0,
      message: message || `训练进程结束 (${code})`
    });
    if (code === 0) console.log(message);
  });
}

function buildStatus() {
  return {
    modelReady,
    demoMode: modelDemoMode,
    modelMessage,
    collecting,
    currentLabel,
    sampleCount,
    training,
    datasetPath: DATASET_PATH
  };
}

function broadcastStatus() {
  broadcast({ type: "system_status", ...buildStatus() });
}

function send(socket, data) {
  if (socket && socket.readyState === WebSocket.OPEN) {
    socket.send(JSON.stringify(data));
  }
}

function broadcast(data, sender = null) {
  const message = JSON.stringify(data);
  wss.clients.forEach((client) => {
    if (client.readyState === WebSocket.OPEN && client !== sender) {
      client.send(message);
    }
  });
}

function toNumber(value) {
  const number = Number(value);
  return Number.isFinite(number) ? number : null;
}

function handleListenError(name, port, error) {
  if (error.code === "EADDRINUSE") {
    console.error(`${name} 端口 ${port} 已被占用。请在旧服务器终端按 Ctrl+C 后重新启动。`);
  } else {
    console.error(`${name} 服务启动失败：${error.message}`);
  }
  process.exitCode = 1;
  setTimeout(() => process.exit(1), 50);
}

function findPython() {
  const candidates = [
    "C:\\ProgramData\\anaconda3\\python.exe",
    "C:\\ProgramData\\miniconda3\\python.exe",
    "python"
  ];
  return candidates.find((candidate) => candidate === "python" || fs.existsSync(candidate));
}

const commandLine = readline.createInterface({ input: process.stdin, output: process.stdout });
commandLine.on("line", (line) => {
  const [command, value] = line.trim().split(/\s+/, 2);
  if (command === "label") {
    const label = String(value || "").toUpperCase();
    if (VALID_LABELS.has(label)) {
      currentLabel = label;
      console.log(`Current label: ${label}`);
      broadcastStatus();
    } else {
      console.log("Labels: NORMAL, WARNING, HIGH_RISK");
    }
  } else if (command === "collect") {
    collecting = value === "on" && Boolean(currentLabel);
    console.log(`Data collection: ${collecting ? "ON" : "OFF"}`);
    broadcastStatus();
  } else if (command === "train") {
    trainModel();
  } else if (command === "status") {
    console.log(buildStatus());
  }
});

startFootSerial();
startPredictor();
console.log("Commands: label NORMAL|WARNING|HIGH_RISK, collect on|off, train, status");
