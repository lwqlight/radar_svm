# radar_svm

### 使用方法

#### 1. 使用 1_collect_data.py 对训练数据进行采集


#### 2. 训练：2_train_svm.py

#### 3. 实时推理：3_realtime_inference.py


# 原理介绍
基于svm方法的雷达点云人体姿态识别项目

基于统计特征的机器学习SVM方法

整个方法分为四个阶段：
第一阶段：底层通信与数据解析 (Data Parsing)

● 硬件配置：配置雷达为侧装模式（0x14），同时开启目标跟踪（0x08）和点云输出（0x06）。

● 核心修正：修正了通信协议的校验算法（从 Sum 改为 XOR），确保了数据包的完整性，解决了数据全为 0 的丢包问题。

● 双流获取：同时读取 Target（目标质心） 和 Point Cloud（散点集合） 数据。

第二阶段：10维特征工程 (Feature Engineering)

为了解决“站立”和“坐下”在简单特征下难以区分的问题，构建了包含几何与统计信息的 10维特征向量：

● 基础特征 (2维)：目标高度 (Z)、目标速度 (V)。
● 几何特征 (4维)：点云宽度 (W)、深度 (D)、面积 (Area)、长宽比 (Ratio) —— 这是区分躺下与站立的关键。
● 统计特征 (4维)：X轴离散度 (Std_X)、Y轴离散度 (Std_Y)、点云数量 (N)、点云密度 (Density)。

第三阶段：模型训练 (Model Training)
● 数据采集：采用“动态采集法”，每种姿态（空/站/坐/跌）录制 500帧（约50秒），包含该姿态下的微小变动，增强模型泛化能力。
● 预处理：使用 StandardScaler 对数据进行标准化，消除不同特征量纲（如速度与面积）的差异。
● 算法：使用 SVM (支持向量机) 配合 RBF (径向基) 核函数，有效处理高维非线性分类边界。

第四阶段：实时推理与后处理 (Inference & Optimization)
● 实时预测：以 10Hz 的频率实时提取特征并输入模型。
● 迟滞滤波 (Hysteresis Filtering)：引入状态机机制，设置确认阈值（常规动作 8 帧，跌倒 3 帧）。
    ● 作用：只有连续 N 帧识别一致时才切换状态，彻底消除了姿态识别中的“闪烁”和“乱跳”现象，实现了稳定输出。

    ```

    ---

    **硬件采集工程：hardware_collection（移动/边缘设备）**

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
        - 增加时间戳字段、采样元信息（设备 ID、固件版本）。
        - 将采集脚本包装为 systemd 服务或容器，便于边缘设备部署并做日志旋转。
        - 添加自动分文件轮转与压缩，避免单个 CSV 过大。


