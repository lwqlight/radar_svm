# 雷达点云人体姿态识别 (Radar SVM)

基于 SVM 方法的雷达点云人体姿态识别项目，通过 10 维特征工程实现对人体站立、坐下、跌倒等姿态的实时检测。

## 项目概述

本项目使用 60GHz 毫米波雷达，结合机器学习（SVM）实现非接触式人体姿态识别。适用于智能家居监护、跌倒检测等场景。

### 特点
- **非接触式检测**：无需佩戴设备，保护用户隐私
- **实时响应**：10Hz 推理频率，低延迟
- **稳定输出**：迟滞滤波消除状态闪烁
- **边缘设备友好**：支持蓝牙遥控采集数据

## 目录结构

```
radar_svm/
├── 1_collect_data.py          # 数据采集主程序
├── 2_train_svm.py             # SVM 模型训练
├── 3_realtime_inference.py    # 实时姿态识别推理
├── feature_extractor.py       # 10 维特征提取器
├── hardware_collection/       # 硬件采集工程（移动设备）
│   ├── 1_collect_data.py      # 蓝牙遥控采集程序
│   ├── config.py              # 配置文件
│   ├── feature_extractor.py   # 特征提取
│   └── read_remote.py         # 遥控调试工具
├── data/                      # 训练数据存放目录
├── weights/                   # 模型权重存放目录
│   ├── radar_svm_model.pkl    # 训练好的 SVM 模型
│   └── radar_scaler.pkl       # 数据标准化器
└── test/                      # 测试脚本
```

## 环境要求

### 硬件
- 60GHz 毫米波雷达（支持侧装模式）
- 计算机/嵌入式设备（Linux 系统）
- （可选）蓝牙遥控模块

### 软件依赖
```bash
pip3 install pyserial numpy pandas scikit-learn joblib
```

或使用 requirements.txt：
```bash
pip3 install -r requirements.txt
```

## 快速开始

### 使用方法

#### 1. 数据采集：1_collect_data.py

连接雷达后运行采集程序：
```bash
python3 1_collect_data.py
```

按照提示输入标签采集数据：
- `0` = 无人/空闲
- `1` = 站立
- `2` = 坐下
- `3` = 跌倒/躺卧

建议每个姿态采集 500 帧以上数据。

#### 2. 模型训练：2_train_svm.py

使用采集的数据训练 SVM 模型：
```bash
python3 2_train_svm.py
```

训练完成后，模型将保存至 `weights/` 目录。

#### 3. 实时推理：3_realtime_inference.py

启动实时姿态识别：
```bash
python3 3_realtime_inference.py
```

程序将持续输出识别结果：
- `🟢 站立` - 人体站立状态
- `🟡 坐下` - 人体坐下状态
- `🚨 跌倒` - 检测到跌倒事件

## 硬件配置

### 串口配置
默认配置位于各脚本顶部，根据实际情况修改：
```python
SERIAL_PORT = '/dev/ttyACM0'  # 雷达串口
BAUD_RATE = 115200            # 波特率
```

### 串口权限
```bash
# 将用户添加到 dialout 组
sudo usermod -a -G dialout $USER

# 或临时使用 sudo（不推荐）
sudo python3 1_collect_data.py
```

## 特征说明

项目提取 **10 维特征** 用于姿态识别：

| 特征 | 说明 | 作用 |
|------|------|------|
| target_z | 目标高度 | 区分姿态的基本特征 |
| target_speed | 目标速度 | 检测运动状态 |
| cloud_width | 点云宽度 | X轴跨度 |
| cloud_depth | 点云深度 | Y轴跨度 |
| cloud_area | 投影面积 | 宽×深，区分站立/躺下的关键 |
| cloud_ratio | 长宽比 | 宽/深，核心判别特征 |
| cloud_std_x | X轴离散度 | 点云分布特征 |
| cloud_std_y | Y轴离散度 | 点云分布特征 |
| cloud_count | 点云数量 | 目标强度指标 |
| cloud_density | 点云密度 | 数量/面积，反映体态 |

## 常见问题

### 串口无法打开
```bash
# 检查串口设备
ls -l /dev/ttyACM*

# 检查串口占用
sudo lsof | grep ttyACM
```

### 特征全为 0
- 检查雷达是否正确配置（侧装模式）
- 确认雷达数据输出正常
- 调整数据清洗阈值（`1_collect_data.py` 第 72 行）

### 模型识别不准确
- 增加训练数据量
- 确保采集数据时姿态典型
- 调整 SVM 参数（`2_train_svm.py` 第 41 行）

### 状态输出闪烁
- 调整滤波窗口 `HISTORY_LEN`（`3_realtime_inference.py` 第 15 行）
- 增大数值可提高稳定性，但会降低响应速度

## 原理介绍

本项目基于统计特征的机器学习 SVM 方法，整个流程分为四个阶段：

### 第一阶段：底层通信与数据解析

- **硬件配置**：配置雷达为侧装模式（0x14），同时开启目标跟踪（0x08）和点云输出（0x06）
- **核心修正**：修正了通信协议的校验算法（从 Sum 改为 XOR），确保数据包完整性
- **双流获取**：同时读取 Target（目标质心）和 Point Cloud（散点集合）数据

### 第二阶段：10 维特征工程

为解决”站立”和”坐下”在简单特征下难以区分的问题，构建了包含几何与统计信息的 10 维特征向量：

- **基础特征 (2 维)**：目标高度 (Z)、目标速度 (V)
- **几何特征 (4 维)**：点云宽度 (W)、深度 (D)、面积 (Area)、长宽比 (Ratio) —— 区分躺下与站立的关键
- **统计特征 (4 维)**：X 轴离散度 (Std_X)、Y 轴离散度 (Std_Y)、点云数量 (N)、点云密度 (Density)

### 第三阶段：模型训练

- **数据采集**：采用”动态采集法”，每种姿态（空/站/坐/跌）录制 500 帧，包含该姿态下的微小变动
- **预处理**：使用 StandardScaler 对数据进行标准化，消除不同特征量纲差异
- **算法**：使用 SVM 配合 RBF（径向基）核函数，处理高维非线性分类边界

### 第四阶段：实时推理与后处理

- **实时预测**：以 10Hz 频率实时提取特征并输入模型
- **迟滞滤波**：引入状态机机制，设置确认阈值（常规动作 8 帧，跌倒 3 帧）
  - 作用：只有连续 N 帧识别一致时才切换状态，消除姿态识别中的”闪烁”和”乱跳”现象

---

## 硬件采集工程：hardware_collection

该模块用于在移动采集设备或边缘硬件上读取雷达与遥控信号。

    - **目的**：在移动采集设备或边缘硬件上可靠地读取雷达与遥控信号，实时提取 10 维特征并保存为训练用 CSV。
    - **关键文件**：
        - [hardware_collection/1_collect_data.py](hardware_collection/1_collect_data.py#L1-L400) — 主采集程序，包含遥控监听、雷达监听、特征提取与 CSV 写入逻辑。
        - [hardware_collection/config.py](hardware_collection/config.py#L1-L200) — 全局配置（串口、波特率、采样参数、标签映射与数据路径）。
        - [hardware_collection/feature_extractor.py](hardware_collection/feature_extractor.py#L1-L200) — 提取 10 维特征并定义 `FEATURE_NAMES`。
        - [hardware_collection/read_remote.py](hardware_collection/read_remote.py#L1-L200) — 用于单独调试遥控信号的辅助脚本。
        - [hardware_collection/data](hardware_collection/data) — 采集到的 CSV 文件存放目录（请在 `.gitignore` 中忽略）。
    - **工作流程（简要）**：
        1. 启动主程序 `1_collect_data.py`，两个线程并行运行：遥控监听线程负责解析遥控器包并将录制命令放入队列；雷达监听线程负责解析点云与目标帧并维护当前状态。
        2. 当接收到经过身份校验的遥控命令时，主线程按 `config.COLLECT_NUM_FRAMES` 连续采样，通过 `feature_extractor.extract_features()` 计算 10 维特征并逐行写入 CSV（文件头为 `FEATURE_NAMES + ['label']`）。
        3. 采集完成后继续等待下一次遥控触发，直到手动停止。
    - **重要配置项（位于 `config.py`）**：
        - `RADAR_PORT`, `RADAR_BAUD`：雷达串口与波特率。
        - `REMOTE_PORT`, `REMOTE_BAUD`：遥控/蓝牙串口与波特率。
        - `COLLECT_NUM_FRAMES`：每次录制的帧数（默认 500）。
        - `COLLECT_DELAY`：帧间延迟（秒），影响采样频率与数据量。
        - `DATA_DIR`, `CSV_PATH`：数据保存目录与文件名。
        - `KEY_MAPPING` / `LABEL_NAMES`：遥控键到标签的映射与可读名称。
    - **数据格式**：输出 CSV 的列顺序为 `feature_extractor.FEATURE_NAMES`：
        - target_z, target_speed, cloud_width, cloud_depth, cloud_area, cloud_ratio, cloud_std_x, cloud_std_y, cloud_count, cloud_density, label
    - **快速运行**：

    ```bash
    pip3 install pyserial numpy pandas
    python3 hardware_collection/1_collect_data.py
    ```

    - **运行前检查与注意事项**：
        - 串口权限：确保用户有 `/dev/ttyACM*` 的读写权限（或加入 `dialout` 组 / 使用 `sudo`）。
        - 调整参数：根据实际雷达输出速率与系统性能调整 `COLLECT_DELAY` 和 `COLLECT_NUM_FRAMES`，避免串口拥堵或数据丢失。
        - 身份验证：遥控帧会校验专属 `TARGET_REMOTE_ID`，若无法触发请检查遥控器 ID 与 `config.py` 设置。
        - 数据量：原始采集量大，请勿将 `hardware_collection/data/` 提交到仓库（见 `.gitignore`）。
    - **常见故障与排查**：
        - 串口无法打开：校验端口号与波特率，检查被其它程序占用。
        - 没有识别到遥控命令：使用 [hardware_collection/read_remote.py](hardware_collection/read_remote.py#L1-L200) 单独调试并打印原始帧。
        - 特征全部为 0：检查雷达是否正确输出点云帧（frame_type 与 payload 解析），或 `feature_extractor` 的阈值过滤是否过严。
    - **扩展建议**：
        - 增加时间戳字段、采样元信息（设备 ID、固件版本）
        - 将采集脚本包装为 systemd 服务或容器，便于边缘设备部署并做日志旋转
        - 添加自动分文件轮转与压缩，避免单个 CSV 过大
