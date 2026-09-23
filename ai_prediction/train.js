const fs = require("fs");
const path = require("path");
const { spawnSync } = require("child_process");

const candidates = [
  process.env.PYTHON_BIN,
  "C:\\ProgramData\\anaconda3\\python.exe",
  "C:\\ProgramData\\miniconda3\\python.exe",
  "python"
].filter(Boolean);

const python = candidates.find((candidate) => candidate === "python" || fs.existsSync(candidate));
const script = path.join(__dirname, "train_model.py");
const result = spawnSync(python, [script], {
  stdio: "inherit",
  windowsHide: true,
  env: { ...process.env, PYTHONUTF8: "1", PYTHONIOENCODING: "utf-8" }
});

if (result.error) {
  console.error(`无法启动 Python：${result.error.message}`);
  process.exit(1);
}
process.exit(result.status ?? 1);
