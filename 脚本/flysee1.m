%% 1. 数据加载与预处理
% 定义表头
opts = delimitedTextImportOptions("NumVariables", 20);
opts.VariableNames = ["timestamp", "angle_roll", "angle_pitch", "angle_yaw", ...
    "gyro_x", "gyro_y", "gyro_z", "acc_x", "acc_y", "acc_z", ...
    "target_roll", "target_pitch", "target_yaw", "target_wx", "target_wy", "target_wz", ...
    "rudder1", "rudder2", "rudder3", "rudder4"];
opts.VariableTypes = ["datetime", repmat("double", 1, 19)];
opts.Delimiter = ",";

% **重要提示:** 请根据您的实际数据格式，取消注释并正确加载您的数据。
% 如果您是直接将数据复制到 MATLAB 命令窗口，请确保它已经赋值给一个名为 'b1' 的 table 变量。
% 假设 b1 已经是一个包含所有数据的 table，并且 timestamp 列是 datetime 类型
data = b2;


%% 2. 数据清洗 (Cleaning)
% A. 剔除重复的时间戳（保留第一次出现的数据）
[~, uniqueIdx] = unique(data.timestamp, 'first');
data = data(uniqueIdx, :);

% B. 处理缺失值 (NaN) - 使用线性插值填充
data{:, 2:end} = fillmissing(data{:, 2:end}, 'linear');

% C. 时间归一化：将绝对时间转换为从0开始的秒数
time_s = seconds(data.timestamp - data.timestamp(1));


%% 3. 可视化 (Visualization)

% --- 图1：姿态角 (Roll, Pitch, Yaw) ---
figure('Color', 'w', 'Name', '姿态角监控'); % 新建一个 figure
plot(time_s, data.angle_roll, 'LineWidth', 1.5, 'DisplayName', 'Roll');
hold on;
plot(time_s, data.angle_pitch, 'LineWidth', 1.5, 'DisplayName', 'Pitch');
plot(time_s, data.angle_yaw, 'LineWidth', 1.5, 'DisplayName', 'Yaw');
title('姿态角监控 (Attitude Angles)');
xlabel('时间 (秒)');
ylabel('角度 (deg)');
legend('Location', 'best');
grid on;
hold off; % 释放 hold

% --- 图2：加速度与角速度动力学 ---
figure('Color', 'w', 'Name', '动力学响应'); % 新建一个 figure
yyaxis left
% 显示原始三轴加速度数据
plot(time_s, data.acc_x, 'Color', [0.8500 0.3250 0.0980], 'LineStyle', '-', 'DisplayName', 'Acc X (Raw)'); % 橙色, 实线
hold on; % 保持左轴的绘图
plot(time_s, data.acc_y, 'Color', [0.9290 0.6940 0.1250], 'LineStyle', '-', 'DisplayName', 'Acc Y (Raw)'); % 黄色, 实线
plot(time_s, data.acc_z, 'Color', [0.4940 0.1840 0.5560], 'LineStyle', '-', 'DisplayName', 'Acc Z (Raw)'); % 紫色, 实线
ylabel('加速度 (m/s²)');
% ylim([-15 15]); % 示例 Y 轴范围，您可能需要根据实际数据调整

yyaxis right
plot(time_s, data.gyro_x, 'DisplayName', 'Gyro X');
plot(time_s, data.gyro_y, 'DisplayName', 'Gyro Y');
plot(time_s, data.gyro_z, 'DisplayName', 'Gyro Z'); % 添加 Gyro Z
title('动力学响应 (Dynamics)');
xlabel('时间 (秒)');
ylabel('角速度 (rad/s)');
% ylim([-5 5]); % 示例 Y 轴范围，您可能需要根据实际数据调整

% 合并两个 yyaxis 的图例
legend('Location', 'best');
grid on;
hold off; % 释放 hold

% --- 图3：舵机反馈与目标控制 ---
figure('Color', 'w', 'Name', '舵机执行状态'); % 新建一个 figure
plot(time_s, data.rudder1, 'DisplayName', 'Rudder 1');
hold on;
plot(time_s, data.rudder2, 'DisplayName', 'Rudder 2');
plot(time_s, data.rudder3, 'DisplayName', 'Rudder 3');
plot(time_s, data.rudder4, 'DisplayName', 'Rudder 4');
title('舵机执行状态 (Rudder Outputs)');
xlabel('时间 (秒)');
ylabel('舵角/占空比'); % 根据您的舵机输出单位修改
legend('NumColumns', 2, 'Location', 'best');
grid on;
hold off; % 释放 hold


%% 4. 数据关联分析 (可选)
% 查看 Roll 角与其目标值的偏差
figure('Color', 'w', 'Name', '姿态跟踪误差');
plot(time_s, data.target_roll, '--k', 'DisplayName', 'Target Roll');
hold on;
plot(time_s, data.angle_roll, 'r', 'LineWidth', 1.2, 'DisplayName', 'Actual Roll');
title('姿态追踪误差分析 (Roll)');
xlabel('时间 (s)');
ylabel('角度 (deg)');
legend;
grid on;
hold off; % 释放 hold