# Peek Balance View

Peek Balance View 是一个面向儿童动态平衡训练的软硬件一体化原型系统。

系统通过足底压力传感器和胸部、腰部 IMU 采集训练过程中的受力与姿态数据，由 Node.js 完成实时数据接收与融合，再使用 Random Forest 模型分析身体稳定性，最后通过网页仪表盘反馈训练状态和风险等级。

## 项目目标

传统平衡训练主要依赖教练观察，难以持续记录细微的姿态变化。Peek Balance View 将传感器数据、实时可视化和机器学习预测结合起来，为训练者提供直观反馈，辅助发现失衡趋势并调整训练动作。

## 系统架构

```text
足底压力传感器 + IMU
          ↓
Arduino 数据采集
          ↓
Node.js WebSocket 数据融合
          ↓
特征提取与标准化
          ↓
Random Forest 模型预测
          ↓
网页仪表盘实时反馈
```

## 核心功能

- 实时采集左脚和右脚压力数据
- 采集胸部、腰部姿态与摆动数据
- 计算左右受力比例、身体摆动差异和稳定性分数
- 通过 WebSocket 实时传输传感器数据
- 使用 Random Forest 判断平衡风险等级
- 展示实时参数、预测置信度和训练趋势
- 支持模拟训练数据，用于演示完整系统流程

## AI Prediction 原理

系统从传感器数据中提取以下特征：

- 胸部摆动幅度 `chestSway`
- 腰部摆动幅度 `waistSway`
- 胸腰摆动差异 `swayDifference`
- 步频 `stepFrequency`
- 身体平衡分数 `imuBalanceScore`
- 左脚压力 `leftPressure`
- 右脚压力 `rightPressure`
- 总压力 `totalPressure`
- 左右压力比例 `lrBalance`
- 前后压力比例 `apBalance`
- 足部摆动幅度 `footSway`
- 足部稳定性分数 `footStabilityScore`

Random Forest 由多棵决策树组成。每棵树独立分析当前特征并给出分类结果，模型再通过多数投票输出最终预测，从而降低单棵决策树过拟合带来的影响。

当前系统输出三个平衡状态等级：

- 低风险
- 中风险
- 高风险

## 当前演示模型

当前原型使用模拟数据验证训练、预测和网页展示流程：

- 样本数量：60 条
- 分类数量：3 类
- 决策树数量：100 棵
- 最大树深：6
- 分裂标准：Gini
- 输入特征：12 项

模拟模型用于验证系统流程，不代表真实用户数据上的最终效果。接入真实训练数据后，可以继续进行交叉验证、参数调优和个体化模型训练。

## 技术栈

- Arduino：硬件数据采集
- MPU6050：姿态与运动数据采集
- 足底压力传感器：脚部受力检测
- Node.js：服务端数据接收与中继
- WebSocket：实时通信
- Python：模型训练与预测
- Scikit-learn：机器学习算法
- Random Forest：平衡风险分类
- HTML / CSS / JavaScript：数据可视化界面

## 推荐项目结构

```text
Peek-Balance-View/
├── server.js                         # Node.js 服务端
├── package.json                      # Node.js 依赖配置
├── ai_prediction/
│   ├── train_model.py                # 模型训练
│   ├── predict_stream.py             # 实时预测
│   ├── generate_demo_model.py        # 生成模拟模型
│   └── balance_model.joblib          # 训练后的模型
├── data/
│   └── demo_balance_dataset.csv      # 模拟训练数据
├── public/                           # 网页界面资源
└── README.md
```

## 安装与运行

### 1. 安装 Node.js 依赖

```bash
npm install
```

### 2. 安装 Python 依赖

```bash
pip install numpy pandas scikit-learn joblib
```

### 3. 生成演示模型

```bash
python ai_prediction/generate_demo_model.py
```

### 4. 启动 Node.js 服务

```bash
node server.js
```

启动后，在浏览器打开：

```text
http://localhost:8080
```

## 数据流说明

Arduino 将传感器数据编码为 JSON，通过串口发送给 Node.js。Node.js 保存最新的 IMU 数据和足底压力数据，完成特征融合后，将统一的 AI 特征向量发送给 Python 预测服务。预测结果随后通过 WebSocket 广播到网页端。

## 项目亮点

1. 将硬件传感器、实时通信、机器学习和网页交互连接成完整闭环。
2. 同时融合足底压力与身体姿态数据，提升对平衡状态的描述能力。
3. 通过网页仪表盘展示实时参数、风险等级和训练趋势。
4. 支持模拟数据训练，便于在缺少真实数据时验证完整产品流程。
5. 系统结构可以继续扩展到个体化训练建议和长期状态追踪。

## 项目定位

Peek Balance View 是用于展示智能平衡训练、传感器数据融合和 AI 预测交互的研究型产品原型，不用于医疗诊断。

## GitHub 简介

基于足底压力与 IMU 数据融合、结合 Random Forest 预测的实时平衡训练系统。

English:

> A real-time balance training system that fuses plantar-pressure and IMU data with Random Forest prediction.
