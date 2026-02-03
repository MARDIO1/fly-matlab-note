%{
对应提示词:

编写一个 MATLAB 脚本用于分析具有循环录制特征的飞机黑匣子 CSV 数据

任务描述：
请编写一个 MATLAB 脚本，从本地选择一个 CSV 文件并进行数据清洗、预处理及可视化。

1. 数据结构：
CSV 文件包含 15 列，表头及顺序如下：
packet_timestamp, statemachine, angle_roll, angle_pitch, angle_yaw, gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z, rudder1, rudder2, rudder3, rudder4

2. 解析逻辑：
文件读取：使用 uigetfile 让用户手动选择路径，并使用 delimitedTextImportOptions 预定义变量类型。
时间戳处理：packet_timestamp 是一个每 10ms 加 1 的数值计数器（不是标准日期格式），请将其设为 double 类型。
循环录制清洗：黑匣子存在循环录制逻辑，数据可能乱序。必须先根据 timestamp 进行 全局升序排序 (sortrows)，然后剔除重复的时间戳 (unique)。
时间归一化：将清洗后的计数器转换为从 0 开始的秒数（步长为 0.01s）。
缺失值处理：对数值列使用线性插值 fillmissing 填充 NaN。
3. 可视化要求：
图1（姿态）：在一个图中绘制 Roll, Pitch, Yaw 随时间（秒）的变化曲线。
图2（动力学）：使用 yyaxis，左轴显示三轴加速度（acc），右轴显示三轴角速度（gyro）。
图3（执行器）：绘制四个舵机输出（rudder1-4）的曲线。
4. 进阶分析（PID 优化基础）：
计算 gyro_x 的一阶导数（角加速度），并将其与 rudder1 的输入绘制在同一个双 Y 轴图中，用于评估控制灵敏度（雅可比矩阵元素估计）。

%}

%% 1. 文件选择与数据加载
clear; clc;

% 弹出文件选择对话框
[file, path] = uigetfile('*.csv', '选择飞控黑匣子数据文件 (.csv)');
if isequal(file, 0), return; end
fullPath = fullfile(path, file);

% 定义数据格式
opts = delimitedTextImportOptions("NumVariables", 15);
opts.VariableNames = ["timestamp", "statemachine", "angle_roll", "angle_pitch", "angle_yaw", ...
    "gyro_x", "gyro_y", "gyro_z", "acc_x", "acc_y", "acc_z", ...
    "rudder1", "rudder2", "rudder3", "rudder4"];
% 注意：这里将 timestamp 改为 double 类型，因为它不是日期而是计数器
opts.VariableTypes = ["double", "double", "double", "double", "double", ...
    "double", "double", "double", "double", "double", "double", ...
    "double", "double", "double", "double"];
opts.Delimiter = ",";

% 读取原始数据
dataRaw = readtable(fullPath, opts);

%% 2. 核心数据清洗 (针对循环录制和计数器格式)

% A. 排序：解决循环录制导致的时间戳乱序问题
% 假设时间戳是从小到大增长的，排序后才能进行后续处理
dataSorted = sortrows(dataRaw, 'timestamp');

% B. 去重：剔除重复的时间戳（保留第一个出现的）
[~, uniqueIdx] = unique(dataSorted.timestamp, 'first');
data = dataSorted(uniqueIdx, :);

% C. 转换时间单位：将计数器转为真实的秒数 (假设每 10ms 加 1)
% 如果 timestamp 是 [1, 2, 3...]，则乘以 0.01 得到秒
% 如果有大幅度的断层（循环录制重置），此处建议查看归一化后的曲线
time_s = (data.timestamp - data.timestamp(1)) * 0.01; 

% D. 处理缺失值 (NaN)
numCols = varfun(@isnumeric, data, 'OutputFormat', 'uniform');
data{:, numCols} = fillmissing(data{:, numCols}, 'linear');

fprintf('原始行数: %d, 清洗后行数: %d\n', height(dataRaw), height(data));

%% 3. 可视化分析

% --- 图1：姿态角 ---
figure('Color', 'w', 'Name', '姿态监控');
plot(time_s, data.angle_roll, 'DisplayName', 'Roll'); hold on;
plot(time_s, data.angle_pitch, 'DisplayName', 'Pitch');
plot(time_s, data.angle_yaw, 'DisplayName', 'Yaw');
title('姿态角 (归一化时间)');
xlabel('时间 (秒)'); ylabel('角度 (deg)');
legend; grid on;

% --- 图2：传感器原始数据 ---
figure('Color', 'w', 'Name', '动力学数据');
subplot(2,1,1);
plot(time_s, [data.acc_x, data.acc_y, data.acc_z]);
title('加速度计 (m/s²)'); ylabel('G'); grid on;
legend('X','Y','Z');

subplot(2,1,2);
plot(time_s, [data.gyro_x, data.gyro_y, data.gyro_z]);
title('角速度 (rad/s 或 deg/s)'); ylabel('Rate'); grid on;
legend('X','Y','Z');

% --- 图3：控制量输出 ---
figure('Color', 'w', 'Name', '舵机输出');
plot(time_s, [data.rudder1, data.rudder2, data.rudder3, data.rudder4]);
title('执行器输出 (Rudder Command)');
xlabel('时间 (秒)'); ylabel('PWM/角度');
legend('R1','R2','R3','R4'); grid on;

%% 4. PID 调优辅助分析：阶跃响应与雅可比
% 计算 Roll 的角加速度并观察其对 Rudder 的响应
% 这是评估 PID 中 P 项和雅可比灵敏度的核心
if height(data) > 10
    dt = 0.01; % 10ms 采样
    % 简单的差分计算角加速度
    ang_accel_x = diff(data.gyro_x) / dt;
    
    figure('Color', 'w', 'Name', '操纵性分析');
    % 绘制一段时间内的舵量与产生的加速度对比
    yyaxis left
    plot(time_s(1:end-1), ang_accel_x, 'DisplayName', 'Roll Accel');
    ylabel('角加速度 (rad/s²)');
    yyaxis right
    plot(time_s, data.rudder1, 'DisplayName', 'Rudder 1');
    ylabel('舵量输入');
    title('控制输入与响应的相位关系');
    grid on; legend;
end