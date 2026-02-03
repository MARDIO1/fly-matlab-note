%% 飞控原始时间戳维度分析工具 (增强版：缺失点断开显示)
clc; clear; close all;

% 1. 弹出对话框选择文件
[file, path] = uigetfile('*.csv', '选择飞控导出文件');
if isequal(file,0)
    disp('未选择文件，程序退出。');
    return; 
end

% 读取数据
opts = detectImportOptions(fullfile(path, file));
T = readtable(fullfile(path, file), opts);

%% 2. 核心清洗逻辑：去重与断裂检测
% 去重
[~, unique_indices] = unique(T.packet_timestamp, 'stable'); 
T_clean = T(unique_indices, :);

% --- 新增：缺失点处理逻辑 ---
% 1. 计算时间间隔
time_raw = T_clean.packet_timestamp;
diffs = diff(time_raw);
median_dt = median(diffs); % 计算正常的时间步长（中位数）
threshold = median_dt * 2.5; % 设定阈值，超过2.5倍步长视为有缺失

% 2. 找到断裂点位置
gap_indices = find(diffs > threshold);

% 3. 在断裂处插入 NaN 数据行
% 我们创建一个和原表结构一样的 NaN 行
nan_row = T_clean(1, :);
nan_row{1, :} = NaN; 

% 倒序插入，防止索引偏移
T_with_gaps = T_clean;
for i = length(gap_indices):-1:1
    idx = gap_indices(i);
    % 插入一条时间戳为中点、数据为NaN的记录
    insert_row = nan_row;
    insert_row.packet_timestamp = time_raw(idx) + median_dt; 
    T_with_gaps = [T_with_gaps(1:idx, :); insert_row; T_with_gaps(idx+1:end, :)];
end

% 更新绘图使用的变量
time_axis = T_with_gaps.packet_timestamp;

fprintf('------------------------------------------\n');
fprintf('处理文件: %s\n', file);
fprintf('检测到断裂点数量: %d\n', length(gap_indices));
fprintf('去重后有效行数: %d\n', height(T_clean));
fprintf('------------------------------------------\n');

%% 3. 创建可视化画布
figure('Color', 'w', 'Name', ['飞控分析: ', file], 'Position', [50 50 1200 900]);
color_set = ["#0072BD", "#D95319", "#EDB120", "#7E2F8E"];

% --- 子图1: 姿态角 ---
ax1 = subplot(4,1,1);
hold on;
plot(time_axis, T_with_gaps.angle_roll, 'Color', color_set(1), 'LineWidth', 1);
plot(time_axis, T_with_gaps.angle_pitch, 'Color', color_set(2), 'LineWidth', 1);
plot(time_axis, T_with_gaps.angle_yaw, 'Color', color_set(3), 'LineWidth', 1);
ylabel('姿态角 (deg)');
legend('Roll', 'Pitch', 'Yaw', 'Location', 'northeastoutside');
grid on; 

% --- 子图2: 角速度 ---
ax2 = subplot(4,1,2);
hold on;
plot(time_axis, T_with_gaps.gyro_x, 'Color', color_set(1));
plot(time_axis, T_with_gaps.gyro_y, 'Color', color_set(2));
plot(time_axis, T_with_gaps.gyro_z, 'Color', color_set(3));
ylabel('角速度 (deg/s)');
legend('Gyro X', 'Gyro Y', 'Gyro Z', 'Location', 'northeastoutside');
grid on;
 
% --- 子图3: 加速度 ---
ax3 = subplot(4,1,3);
hold on;
plot(time_axis, T_with_gaps.acc_x, 'Color', color_set(1));
plot(time_axis, T_with_gaps.acc_y, 'Color', color_set(2));
plot(time_axis, T_with_gaps.acc_z, 'Color', color_set(3));
ylabel('加速度 (mg)');
legend('Acc X', 'Acc Y', 'Acc Z', 'Location', 'northeastoutside');
grid on;

% --- 子图4: 舵面输出 ---
ax4 = subplot(4,1,4);
hold on;
plot(time_axis, T_with_gaps.rudder1, 'Color', color_set(1));
plot(time_axis, T_with_gaps.rudder2, 'Color', color_set(2));
plot(time_axis, T_with_gaps.rudder3, 'Color', color_set(3));
plot(time_axis, T_with_gaps.rudder4, 'Color', color_set(4));
ylabel('舵面输出');
xlabel('原始时间戳 (Raw Timestamp)'); 
legend('R1', 'R2', 'R3', 'R4', 'Location', 'northeastoutside');
grid on;

%% 4. 交互功能：关联缩放
linkaxes([ax1, ax2, ax3, ax4], 'x');

% 自动缩放到起始有数据的区域
valid_idx = find(~isnan(time_axis), 1, 'first');
xlim([time_axis(valid_idx), time_axis(valid_idx) + (median_dt * 500)]);

disp('脚本运行完毕。图表中直线中断处即为原始数据缺失/丢包处。');