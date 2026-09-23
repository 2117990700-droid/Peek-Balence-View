# PEEK Balance View — AI 使用说明

## 1. 启动

关闭 Arduino IDE 的串口监视器，然后在项目目录运行：

```powershell
npm start
```

浏览器打开：

```text
http://localhost:8080
```

默认足底 Arduino 端口是 `COM3`。如果实际端口不同，可修改 `server.js` 中的 `FOOT_PORT`，或先运行：

```powershell
$env:FOOT_PORT="COM5"
npm start
```

## 2. 安装 Python 依赖

只需安装一次：

```powershell
C:\ProgramData\anaconda3\python.exe -m pip install -r ai_prediction/requirements.txt
```

## 3. 采集数据

网页中选择当前实验标签，再点击“开始采集”。

- `NORMAL`：正常稳定站立或行走
- `WARNING`：轻度不稳定的受控实验
- `HIGH_RISK`：明显不稳定的受控模拟

数据保存在：

```text
data/balance_dataset.csv
```

采集会在 IMU 数据和足底数据都存在、且足底数据未超过 15 秒时进行。每种标签建议至少 50 条，正式展示建议每种标签采集 200 条以上，并涵盖不同受试者和动作。

如果暂时没有真实数据，可生成明确标记的合成演示模型：

```powershell
npm run demo-model
```

它会生成 `data/demo_balance_dataset.csv`，不会写入真实数据文件。页面会显示“演示模型（合成样本）”。

## 4. 训练模型

数据足够后点击网页中的“训练模型”，或者运行：

```powershell
npm run train
```

生成文件：

```text
ai_prediction/balance_model.joblib
ai_prediction/metrics.json
```

模型完成后无需重启服务器；预测进程会自动加载新模型。

## 5. 当前限制

没有训练数据时，系统只显示融合特征和 Arduino 规则风险，AI 区域显示“模型未训练”。这是预期行为。代码不会使用虚构数据制造预测结果。
