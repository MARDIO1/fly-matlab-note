%% =====================================================================
%  X型翼飞行器 - 物理约束雅可比矩阵拟合 + PID调参分析
%  舵面顺序: [FR, FL, BL, BR] (前右, 前左, 后左, 后右)
%% =====================================================================

clear; clc; close all;

%% 1. 加载多文件
[files, path] = uigetfile('*.csv', '选择黑匣子CSV文件（可多选）', 'MultiSelect', 'on');

if isequal(files, 0)
    error('未选择文件');
end

if ischar(files)
    files = {files};
end

nFiles = length(files);
fprintf('===== X型翼雅可比矩阵拟合 + PID分析 =====\n');
fprintf('已选择 %d 个文件\n\n', nFiles);

%% 2. 列名配置 (根据您的数据格式)
% 舵面列名
rudderCols = {'rudder1', 'rudder2', 'rudder3', 'rudder4'};  % FR, FL, BL, BR

% 姿态角度列名 (度)
angleCols = {'angle_roll', 'angle_pitch', 'angle_yaw'};

% 角速度列名 (rad/s)
gyroCols = {'gyro_x', 'gyro_y', 'gyro_z'};

% 加速度列名
accCols = {'acc_x', 'acc_y', 'acc_z'};

%% 3. 合并所有文件数据
allData = [];

for i = 1:nFiles
    filePath = fullfile(path, files{i});
    try
        T = readtable(filePath);
        fprintf('文件 %d: %s (%d行)\n', i, files{i}, height(T));
        allData = [allData; T];
    catch ME
        fprintf('文件 %d 读取失败: %s\n', i, ME.message);
    end
end

fprintf('\n合并后总数据: %d 行\n\n', height(allData));

%% 4. 提取数据
% 时间
time_ms = allData.packet_timestamp;
time_s = (time_ms - time_ms(1)) / 1000;
dt = median(diff(time_s));
Fs = 1/dt;
fprintf('采样频率: %.1f Hz\n', Fs);

% 舵面 [FR, FL, BL, BR]
delta_FR = allData.rudder1;
delta_FL = allData.rudder2;
delta_BL = allData.rudder3;
delta_BR = allData.rudder4;

% 姿态角度 (度)
angle_roll  = allData.angle_roll;
angle_pitch = allData.angle_pitch;
angle_yaw   = allData.angle_yaw;

% 角速度 (rad/s -> deg/s)
gyro_roll  = rad2deg(allData.gyro_x);
gyro_pitch = rad2deg(allData.gyro_y);
gyro_yaw   = rad2deg(allData.gyro_z);

% 加速度
acc_x = allData.acc_x;
acc_y = allData.acc_y;
acc_z = allData.acc_z;

% 状态机
state = allData.statemachine;

%% 5. 数据清洗与分段
% 找出AUTO模式数据 (state高4位 = 3)
auto_mask = bitshift(state, -4) == 3;
fprintf('AUTO模式数据点: %d (%.1f%%)\n', sum(auto_mask), 100*sum(auto_mask)/length(state));

% 只分析AUTO模式
if sum(auto_mask) > 100
    idx = find(auto_mask);
else
    idx = 1:length(time_s);
    fprintf('警告: AUTO模式数据不足，使用全部数据\n');
end

%% ==================== 第一部分: 雅可比矩阵拟合 ====================
fprintf('\n===== 雅可比矩阵拟合 =====\n');

% 构建输入矩阵 (微分)
dDelta_FR = [0; diff(delta_FR)];
dDelta_FL = [0; diff(delta_FL)];
dDelta_BL = [0; diff(delta_BL)];
dDelta_BR = [0; diff(delta_BR)];

% 构建输出矩阵 (角速度变化率)
dGyro_roll  = [0; diff(gyro_roll)];
dGyro_pitch = [0; diff(gyro_pitch)];
dGyro_yaw   = [0; diff(gyro_yaw)];

% 组合输入 (对称约束)
diff_front = dDelta_FR - dDelta_FL;    % 前翼差动 -> Roll
diff_back  = dDelta_BR - dDelta_BL;    % 后翼差动 -> Roll
sum_back   = dDelta_BR + dDelta_BL;    % 后翼同动 -> Pitch
sum_front  = dDelta_FR + dDelta_FL;    % 前翼同动 -> Pitch (较小)
diag_diff  = (dDelta_FR + dDelta_BL) - (dDelta_FL + dDelta_BR);  % 对角差 -> Yaw

% 有效数据索引 (排除静止)
valid_idx = abs(dDelta_FR) + abs(dDelta_FL) + abs(dDelta_BL) + abs(dDelta_BR) > 0.1;
valid_idx = valid_idx & idx';
fprintf('有效数据点: %d\n', sum(valid_idx));

%% 物理对称约束拟合
% Roll = a*(FR-FL) + b*(BR-BL)
X_roll = [diff_front(valid_idx), diff_back(valid_idx)];
y_roll = dGyro_roll(valid_idx);
coef_roll = X_roll \ y_roll;
pred_roll = X_roll * coef_roll;
R2_roll = 1 - sum((y_roll - pred_roll).^2) / sum((y_roll - mean(y_roll)).^2);

% Pitch = c*(BR+BL) + d*(FR+FL)
X_pitch = [sum_back(valid_idx), sum_front(valid_idx)];
y_pitch = dGyro_pitch(valid_idx);
coef_pitch = X_pitch \ y_pitch;
pred_pitch = X_pitch * coef_pitch;
R2_pitch = 1 - sum((y_pitch - pred_pitch).^2) / sum((y_pitch - mean(y_pitch)).^2);

% Yaw = e*[(FR+BL) - (FL+BR)]
X_yaw = diag_diff(valid_idx);
y_yaw = dGyro_yaw(valid_idx);
coef_yaw = X_yaw \ y_yaw;
pred_yaw = X_yaw * coef_yaw;
R2_yaw = 1 - sum((y_yaw - pred_yaw).^2) / sum((y_yaw - mean(y_yaw)).^2);

%% 构建对称雅可比矩阵
a = coef_roll(1);   % 前翼差动对Roll
b = coef_roll(2);   % 后翼差动对Roll
c = coef_pitch(1);  % 后翼同动对Pitch
d = coef_pitch(2);  % 前翼同动对Pitch
e = coef_yaw(1);    % 对角差对Yaw

%        [FR,  FL,  BL,  BR]
J_sym = [+a,  -a,  -b,  +b;    % Roll
         +d,  +d,  +c,  +c;    % Pitch
         +e,  -e,  +e,  -e];   % Yaw

fprintf('\n===== 对称约束雅可比矩阵 =====\n');
fprintf('系数: a=%.4f, b=%.4f, c=%.4f, d=%.4f, e=%.4f\n', a, b, c, d, e);
fprintf('R²: Roll=%.3f, Pitch=%.3f, Yaw=%.3f\n', R2_roll, R2_pitch, R2_yaw);

fprintf('\n       FR        FL        BL        BR\n');
labels = {'Roll', 'Pitch', 'Yaw'};
for i = 1:3
    fprintf('%s:  %+.4f   %+.4f   %+.4f   %+.4f\n', labels{i}, J_sym(i,:));
end

%% 归一化雅可比矩阵
J_norm = J_sym ./ max(abs(J_sym), [], 2);
fprintf('\n===== 归一化雅可比矩阵 =====\n');
fprintf('       FR        FL        BL        BR\n');
for i = 1:3
    fprintf('%s:  %+.4f   %+.4f   %+.4f   %+.4f\n', labels{i}, J_norm(i,:));
end

%% ==================== 第二部分: PID响应分析 ====================
fprintf('\n\n===== PID响应分析 =====\n');

% 您当前的PID参数
PID_params = struct(...
    'roll_outer_kp', 6.0, ...
    'roll_inner_kp', 0.15, 'roll_inner_ki', 0.0, 'roll_inner_kd', 0.008, ...
    'pitch_outer_kp', 6.0, ...
    'pitch_inner_kp', 0.15, 'pitch_inner_ki', 0.0, 'pitch_inner_kd', 0.008, ...
    'yaw_outer_kp', 2.0, ...
    'yaw_inner_kp', 0.1, 'yaw_inner_ki', 0.0, 'yaw_inner_kd', 0.0);

fprintf('当前PID参数:\n');
fprintf('Roll:  外环Kp=%.2f, 内环Kp=%.3f Ki=%.3f Kd=%.4f\n', ...
    PID_params.roll_outer_kp, PID_params.roll_inner_kp, ...
    PID_params.roll_inner_ki, PID_params.roll_inner_kd);
fprintf('Pitch: 外环Kp=%.2f, 内环Kp=%.3f Ki=%.3f Kd=%.4f\n', ...
    PID_params.pitch_outer_kp, PID_params.pitch_inner_kp, ...
    PID_params.pitch_inner_ki, PID_params.pitch_inner_kd);
fprintf('Yaw:   外环Kp=%.2f, 内环Kp=%.3f Ki=%.3f Kd=%.4f\n', ...
    PID_params.yaw_outer_kp, PID_params.yaw_inner_kp, ...
    PID_params.yaw_inner_ki, PID_params.yaw_inner_kd);

%% 检测阶跃响应段
min_step_size = 3.0;  % 最小阶跃幅度 (度)
min_duration = 20;     % 最小持续采样点

% 检测Roll阶跃
angle_roll_filt = movmean(angle_roll, 5);
roll_diff = [0; diff(angle_roll_filt)];
roll_steps = find(abs(roll_diff) > min_step_size / 10);

% 检测Pitch阶跃  
angle_pitch_filt = movmean(angle_pitch, 5);
pitch_diff = [0; diff(angle_pitch_filt)];
pitch_steps = find(abs(pitch_diff) > min_step_size / 10);

fprintf('\n检测到的阶跃点: Roll=%d, Pitch=%d\n', length(roll_steps), length(pitch_steps));

%% 分析阶跃响应特性
function [rise_time, overshoot, settling_time, steady_error] = analyzeStep(time, signal, step_idx, window)
    if step_idx + window > length(signal)
        window = length(signal) - step_idx;
    end
    
    t = time(step_idx:step_idx+window) - time(step_idx);
    y = signal(step_idx:step_idx+window);
    
    y0 = y(1);
    yf = y(end);
    dy = yf - y0;
    
    if abs(dy) < 1
        rise_time = NaN; overshoot = NaN; settling_time = NaN; steady_error = NaN;
        return;
    end
    
    % 上升时间 (10% -> 90%)
    y10 = y0 + 0.1 * dy;
    y90 = y0 + 0.9 * dy;
    idx10 = find(y >= y10, 1);
    idx90 = find(y >= y90, 1);
    if isempty(idx10), idx10 = 1; end
    if isempty(idx90), idx90 = length(t); end
    rise_time = t(idx90) - t(idx10);
    
    % 超调量
    if dy > 0
        peak = max(y);
    else
        peak = min(y);
    end
    overshoot = 100 * abs(peak - yf) / abs(dy);
    
    % 调节时间 (5%误差带)
    error_band = 0.05 * abs(dy);
    settled = abs(y - yf) < error_band;
    settled_idx = find(settled, 1, 'first');
    if isempty(settled_idx)
        settling_time = t(end);
    else
        settling_time = t(settled_idx);
    end
    
    % 稳态误差
    steady_error = abs(y(end) - yf);
end

%% 分析多个阶跃响应
window_size = round(2.0 * Fs);  % 2秒窗口

roll_metrics = [];
for i = 1:min(10, length(roll_steps))
    [rt, os, st, se] = analyzeStep(time_s, angle_roll, roll_steps(i), window_size);
    if ~isnan(rt)
        roll_metrics = [roll_metrics; rt, os, st, se];
    end
end

pitch_metrics = [];
for i = 1:min(10, length(pitch_steps))
    [rt, os, st, se] = analyzeStep(time_s, angle_pitch, pitch_steps(i), window_size);
    if ~isnan(rt)
        pitch_metrics = [pitch_metrics; rt, os, st, se];
    end
end

%% 输出响应分析结果
fprintf('\n===== 阶跃响应分析 =====\n');
if ~isempty(roll_metrics)
    fprintf('Roll轴 (平均值, N=%d):\n', size(roll_metrics,1));
    fprintf('  上升时间: %.3f s\n', mean(roll_metrics(:,1)));
    fprintf('  超调量:   %.1f %%\n', mean(roll_metrics(:,2)));
    fprintf('  调节时间: %.3f s\n', mean(roll_metrics(:,3)));
else
    fprintf('Roll轴: 未检测到有效阶跃响应\n');
end

if ~isempty(pitch_metrics)
    fprintf('Pitch轴 (平均值, N=%d):\n', size(pitch_metrics,1));
    fprintf('  上升时间: %.3f s\n', mean(pitch_metrics(:,1)));
    fprintf('  超调量:   %.1f %%\n', mean(pitch_metrics(:,2)));
    fprintf('  调节时间: %.3f s\n', mean(pitch_metrics(:,3)));
else
    fprintf('Pitch轴: 未检测到有效阶跃响应\n');
end

%% ==================== 第三部分: 频域分析 ====================
fprintf('\n===== 频域分析 =====\n');

% 计算角速度频谱
nfft = 2^nextpow2(length(gyro_roll));
f = Fs * (0:(nfft/2)) / nfft;

% Roll角速度频谱
Y_roll = fft(gyro_roll - mean(gyro_roll), nfft);
P_roll = abs(Y_roll(1:nfft/2+1)) / length(gyro_roll);

% Pitch角速度频谱
Y_pitch = fft(gyro_pitch - mean(gyro_pitch), nfft);
P_pitch = abs(Y_pitch(1:nfft/2+1)) / length(gyro_pitch);

% 找主频
[~, idx_roll] = max(P_roll(2:end)); 
[~, idx_pitch] = max(P_pitch(2:end));
fprintf('Roll 主频: %.2f Hz\n', f(idx_roll+1));
fprintf('Pitch 主频: %.2f Hz\n', f(idx_pitch+1));

%% ==================== 第四部分: 轴间耦合分析 ====================
fprintf('\n===== 轴间耦合分析 =====\n');

% 计算相关系数
corr_roll_pitch = corrcoef(gyro_roll, gyro_pitch);
corr_roll_yaw = corrcoef(gyro_roll, gyro_yaw);
corr_pitch_yaw = corrcoef(gyro_pitch, gyro_yaw);

fprintf('Roll-Pitch 相关: %.3f\n', corr_roll_pitch(1,2));
fprintf('Roll-Yaw 相关:   %.3f\n', corr_roll_yaw(1,2));
fprintf('Pitch-Yaw 相关:  %.3f\n', corr_pitch_yaw(1,2));

coupling_threshold = 0.3;
if abs(corr_roll_pitch(1,2)) > coupling_threshold
    fprintf('⚠️  Roll-Pitch耦合较强，考虑解耦控制\n');
end
if abs(corr_roll_yaw(1,2)) > coupling_threshold
    fprintf('⚠️  Roll-Yaw耦合较强，考虑解耦控制\n');
end

%% ==================== 第五部分: PID调参建议 ====================
fprintf('\n===== PID调参建议 =====\n');

% 基于响应分析给出建议
if ~isempty(roll_metrics)
    avg_rise = mean(roll_metrics(:,1));
    avg_overshoot = mean(roll_metrics(:,2));
    
    fprintf('\nRoll轴:\n');
    if avg_rise > 0.5
        fprintf('  → 上升时间偏长(%.2fs)，建议增大外环Kp: %.1f -> %.1f\n', ...
            avg_rise, PID_params.roll_outer_kp, PID_params.roll_outer_kp * 1.3);
    elseif avg_rise < 0.15
        fprintf('  → 响应过快(%.2fs)，可能有震荡风险，建议减小外环Kp\n', avg_rise);
    else
        fprintf('  → 上升时间良好(%.2fs)\n', avg_rise);
    end
    
    if avg_overshoot > 25
        fprintf('  → 超调过大(%.1f%%)，建议增大内环Kd: %.4f -> %.4f\n', ...
            avg_overshoot, PID_params.roll_inner_kd, PID_params.roll_inner_kd * 1.5);
    elseif avg_overshoot < 5
        fprintf('  → 超调良好(%.1f%%)\n', avg_overshoot);
    end
end

if ~isempty(pitch_metrics)
    avg_rise = mean(pitch_metrics(:,1));
    avg_overshoot = mean(pitch_metrics(:,2));
    
    fprintf('\nPitch轴:\n');
    if avg_rise > 0.5
        fprintf('  → 上升时间偏长(%.2fs)，建议增大外环Kp: %.1f -> %.1f\n', ...
            avg_rise, PID_params.pitch_outer_kp, PID_params.pitch_outer_kp * 1.3);
    else
        fprintf('  → 上升时间良好(%.2fs)\n', avg_rise);
    end
    
    if avg_overshoot > 25
        fprintf('  → 超调过大(%.1f%%)，建议增大内环Kd: %.4f -> %.4f\n', ...
            avg_overshoot, PID_params.pitch_inner_kd, PID_params.pitch_inner_kd * 1.5);
    end
end

%% ==================== 可视化 ====================
%% 图1: 数据总览
figure('Name', '数据总览', 'Position', [50 50 1400 900]);

subplot(4,1,1);
plot(time_s, angle_roll, 'r-', 'LineWidth', 0.8); hold on;
plot(time_s, angle_pitch, 'g-', 'LineWidth', 0.8);
plot(time_s, angle_yaw, 'b-', 'LineWidth', 0.8);
ylabel('角度 (°)');
legend('Roll', 'Pitch', 'Yaw', 'Location', 'best');
title('姿态角度');
grid on;

subplot(4,1,2);
plot(time_s, gyro_roll, 'r-', 'LineWidth', 0.8); hold on;
plot(time_s, gyro_pitch, 'g-', 'LineWidth', 0.8);
plot(time_s, gyro_yaw, 'b-', 'LineWidth', 0.8);
ylabel('角速度 (°/s)');
legend('Roll', 'Pitch', 'Yaw', 'Location', 'best');
title('角速度');
grid on;

subplot(4,1,3);
plot(time_s, delta_FR, 'LineWidth', 0.8); hold on;
plot(time_s, delta_FL, 'LineWidth', 0.8);
plot(time_s, delta_BL, 'LineWidth', 0.8);
plot(time_s, delta_BR, 'LineWidth', 0.8);
ylabel('舵面 (°)');
legend('FR', 'FL', 'BL', 'BR', 'Location', 'best');
title('舵面输出');
grid on;

subplot(4,1,4);
plot(time_s, state, 'k-', 'LineWidth', 1);
ylabel('状态机');
xlabel('时间 (s)');
title('状态机');
grid on;

%% 图2: 雅可比矩阵可视化
figure('Name', '雅可比矩阵', 'Position', [100 100 900 400]);

subplot(1,2,1);
imagesc(J_sym);
colorbar;
set(gca, 'XTick', 1:4, 'XTickLabel', {'FR', 'FL', 'BL', 'BR'});
set(gca, 'YTick', 1:3, 'YTickLabel', {'Roll', 'Pitch', 'Yaw'});
title('雅可比矩阵 (原始)');
for i = 1:3
    for j = 1:4
        text(j, i, sprintf('%.3f', J_sym(i,j)), ...
            'HorizontalAlignment', 'center', 'Color', 'w', 'FontWeight', 'bold');
    end
end

subplot(1,2,2);
imagesc(J_norm);
colorbar;
caxis([-1, 1]);
set(gca, 'XTick', 1:4, 'XTickLabel', {'FR', 'FL', 'BL', 'BR'});
set(gca, 'YTick', 1:3, 'YTickLabel', {'Roll', 'Pitch', 'Yaw'});
title('归一化雅可比矩阵');
for i = 1:3
    for j = 1:4
        text(j, i, sprintf('%.2f', J_norm(i,j)), ...
            'HorizontalAlignment', 'center', 'Color', 'w', 'FontWeight', 'bold');
    end
end

%% 图3: 频谱分析
figure('Name', '频谱分析', 'Position', [150 150 1000 400]);

subplot(1,2,1);
semilogy(f, P_roll, 'r-', 'LineWidth', 1);
xlabel('频率 (Hz)');
ylabel('幅值');
title('Roll角速度频谱');
xlim([0, min(20, Fs/2)]);
grid on;

subplot(1,2,2);
semilogy(f, P_pitch, 'g-', 'LineWidth', 1);
xlabel('频率 (Hz)');
ylabel('幅值');
title('Pitch角速度频谱');
xlim([0, min(20, Fs/2)]);
grid on;

%% 图4: 轴间耦合散点图
figure('Name', '轴间耦合', 'Position', [200 200 1200 400]);

subplot(1,3,1);
scatter(gyro_roll, gyro_pitch, 1, 'b');
xlabel('Roll Rate (°/s)');
ylabel('Pitch Rate (°/s)');
title(sprintf('Roll-Pitch (r=%.2f)', corr_roll_pitch(1,2)));
grid on;

subplot(1,3,2);
scatter(gyro_roll, gyro_yaw, 1, 'r');
xlabel('Roll Rate (°/s)');
ylabel('Yaw Rate (°/s)');
title(sprintf('Roll-Yaw (r=%.2f)', corr_roll_yaw(1,2)));
grid on;

subplot(1,3,3);
scatter(gyro_pitch, gyro_yaw, 1, 'g');
xlabel('Pitch Rate (°/s)');
ylabel('Yaw Rate (°/s)');
title(sprintf('Pitch-Yaw (r=%.2f)', corr_pitch_yaw(1,2)));
grid on;

%% 生成C代码
fprintf('\n===== 生成C代码 =====\n');
fprintf('// 雅可比矩阵 (由MATLAB拟合生成)\n');
fprintf('// 舵面顺序: [FR, FL, BL, BR]\n');
fprintf('static const float Jacobian_Matrix[3][4] = {\n');
fprintf('    {%+.6ff, %+.6ff, %+.6ff, %+.6ff},  // Roll\n', J_sym(1,:));
fprintf('    {%+.6ff, %+.6ff, %+.6ff, %+.6ff},  // Pitch\n', J_sym(2,:));
fprintf('    {%+.6ff, %+.6ff, %+.6ff, %+.6ff}   // Yaw\n', J_sym(3,:));
fprintf('};\n');

fprintf('\n===== 分析完成 =====\n');