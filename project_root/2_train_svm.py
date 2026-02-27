import pandas as pd
from sklearn.model_selection import train_test_split
from sklearn.svm import SVC
from sklearn.preprocessing import StandardScaler
from sklearn.metrics import classification_report, accuracy_score
import joblib
import os
import feature_extractor  # 【关键】导入特征定义模块，保证和采集、推理完全一致

# 1. 读取数据
csv_file = "./data/radar_training_data.csv" # 确保路径和采集时一致
print(f"正在读取 {csv_file} ...")

if not os.path.exists(csv_file):
    print(f"❌ 错误：找不到文件 {csv_file}，请检查路径或先运行采集脚本。")
    exit()

df = pd.read_csv(csv_file)

# 2. 准备特征 (X) 和 标签 (y)
# 【核心修改】不再手写列名，而是直接使用 feature_extractor.FEATURE_NAMES
# 这样不仅包含了所有 10 个新特征，而且以后改特征不用到处改代码
try:
    X = df[feature_extractor.FEATURE_NAMES]
    y = df["label"]
except KeyError as e:
    print(f"❌ 数据列名不匹配！CSV中缺少列: {e}")
    print("请删除旧的 CSV 文件并重新运行 1_collect_data.py 采集数据。")
    exit()

# 3. 数据划分
X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, random_state=42)

# 4. 数据标准化
scaler = StandardScaler()
X_train_scaled = scaler.fit_transform(X_train)
X_test_scaled = scaler.transform(X_test)

# 5. 训练 SVM 模型
print(f"正在训练 SVM 模型 (特征维度: {X.shape[1]})...")
svm_model = SVC(kernel='rbf', C=1.0, gamma='scale', probability=True)
svm_model.fit(X_train_scaled, y_train)

# 6. 评估模型
y_pred = svm_model.predict(X_test_scaled)
print("\n--- 模型评估报告 ---")
print(f"准确率: {accuracy_score(y_test, y_pred):.2f}")
print(classification_report(y_test, y_pred))

# 7. 保存模型
# 确保权重目录存在
weights_dir = "./weights"
if not os.path.exists(weights_dir):
    os.makedirs(weights_dir)

model_path = os.path.join(weights_dir, 'radar_svm_model.pkl')
scaler_path = os.path.join(weights_dir, 'radar_scaler.pkl')

joblib.dump(svm_model, model_path)
joblib.dump(scaler, scaler_path)
print(f"\n✅ 模型已保存至: {weights_dir}")
print("你可以运行 3_realtime_inference.py 来加载模型并进行实时推理了！")
print("10维度不一定够用，后续可以考虑增加更多特征（如点云分布特征、历史统计特征等）来提升模型性能。" \
"增加时间维度的特征（如移动平均、差分等）通常对动态事件（如跌倒）非常有帮助。")
