%% =====================================================================
%  X型翼飞控PID调参分析系统
%  基于您的串级PID控制架构:
%  目标角度 -> 外环(角度PID) -> 目标角速度 -> 内环(角速度PID) -> 力矩 -> 雅可比分配 -> 舵面
%  =====================================================================
clear; clc; close all;

%% ==================== 当前PID参数 (从您的代码提取) ====================
% 外环 (角度环) - 输入:角度误差(deg), 输出:目标角速度(rad/s)
PID_outer.roll  = struct('Kp', 0.1, 'Ki', 0, 'Kd', 0, 'IntMax', 0,   'OutMax', 1.5);
PID_outer.pitch = struct('Kp', 0.1, 'Ki', 0, 'Kd', 0, 'IntMax', 0,   'OutMax', 2.0);
PID_outer.yaw   = struct('Kp', 0.1, 'Ki', 0, 'Kd', 0, 'IntMax', 0,   'OutMax', 2.0);

% 内环 (角速度环) - 输入:角速度误差(rad/s), 输出:力矩权重
PID_inner.roll  = struct('Kp', 10, 'Ki', 0, 'Kd', 0, 'IntMax', 50,  'OutMax', 60);
PID_inner.pitch = struct('Kp', 10, 'Ki', 0, 'Kd', 0, 'IntMax', 200, 'OutMax', 60);
PID_inner.yaw   = struct('Kp', 10, 'Ki', 0, 'Kd', 0, 'IntMax', 50,  'OutMax', 60);

% 雅可比矩阵 (您拟合的结果)
JM = [+1.00, -1.00, -0.50, +0.50;   % Roll
      +0.00, +0.00, +0.35, +0.35;   % Pitch
      +0.00, -0.00, +0.20, -0.20];  % Yaw

% 控制周期
dt = 0.005;  % 5ms, 200Hz

%% ==================== 1. 多文件数据加载 ====================
fprintf('===== PID调参分析系统 =====\n\n');

[files, path] = uigetfile('*.csv', '选择黑匣子数据文件', 'MultiSelect', 'on');
if isequal(files, 0)
    error('未选择文件');
end
if ischar(files)
    files = {files};
end

% 合并所有数据
dataAll = [];
for i = 1:length(files)
    filePath = fullfile(path, files{i});
    T = readtable(filePath, 'VariableNamingRule', 'preserve');
    fprintf('文件 %d: %s (%d行)\n', i, files{i}, height(T));
    dataAll = [dataAll; T];
end
fprintf('\n合并后总数据: %d 行\n\n', height(dataAll));

%% ==================== 2. 提取关键变量 ====================
% 根据您的BlackBox结构提取数据
try
    % 时间
    if ismember('timestamp', dataAll.Properties.VariableNames)
        time_raw = dataAll.timestamp;
        time_s = (time_raw - time_raw(1)) * dt;  % 转换为秒
    else
        time_s = (0:height(dataAll)-1)' * dt;
    end
    
    % 实际姿态角 (世界系)
    angle_roll  = dataAll.angle_roll;
    angle_pitch = dataAll.angle_pitch;
    angle_yaw   = dataAll.angle_yaw;
    
    % 实际角速度 (机体系)
    gyro_x = dataAll.gyro_x;  % roll rate
    gyro_y = dataAll.gyro_y;  % pitch rate
    gyro_z = dataAll.gyro_z;  % yaw rate
    
    % 目标姿态 (世界系)
    target_roll  = dataAll.target_roll;
    target_pitch = dataAll.target_pitch;
    target_yaw   = dataAll.target_yaw;
    
    % 舵面角度
    rudder_FR = dataAll.rudder_FR;
    rudder_FL = dataAll.rudder_FL;
    rudder_BL = dataAll.rudder_BL;
    rudder_BR = dataAll.rudder_BR;
    
    fprintf('数据提取成功!\n\n');
    
catch ME
    fprintf('警告: 部分变量提取失败 - %s\n', ME.message);
    fprintf('可用列名:\n');
    disp(dataAll.Properties.VariableNames');
    return;
end

%% ==================== 3. 计算控制过程中间变量 ====================
% 这些是您代码中的中间变量，需要从数据反推

N = length(time_s);

% 3.1 外环: 角度误差 (简化计算，实际您用四元数)
err_roll  = target_roll - angle_roll;
err_pitch = target_pitch - angle_pitch;
err_yaw   = wrapTo180(target_yaw - angle_yaw);  % 处理角度跨越

% 3.2 外环输出: 目标角速度 (rad/s)
target_gyro_roll  = PID_outer.roll.Kp  * err_roll;
target_gyro_pitch = PID_outer.pitch.Kp * err_pitch;
target_gyro_yaw   = PID_outer.yaw.Kp   * err_yaw;

% 限幅
target_gyro_roll  = max(min(target_gyro_roll,  PID_outer.roll.OutMax),  -PID_outer.roll.OutMax);
target_gyro_pitch = max(min(target_gyro_pitch, PID_outer.pitch.OutMax), -PID_outer.pitch.OutMax);
target_gyro_yaw   = max(min(target_gyro_yaw,   PID_outer.yaw.OutMax),   -PID_outer.yaw.OutMax);

% 3.3 内环: 角速度误差
err_gyro_roll  = target_gyro_roll  - gyro_x;
err_gyro_pitch = target_gyro_pitch - gyro_y;
err_gyro_yaw   = target_gyro_yaw   - gyro_z;

% 3.4 内环输出: 力矩权重
torque_roll  = PID_inner.roll.Kp  * err_gyro_roll;
torque_pitch = PID_inner.pitch.Kp * err_gyro_pitch;
torque_yaw   = PID_inner.yaw.Kp   * err_gyro_yaw;

%% ==================== 4. 响应特性分析 ====================
fprintf('===== 4. 响应特性分析 =====\n\n');

figure('Name', 'PID响应分析', 'Position', [50, 50, 1400, 900]);

% --- 4.1 Roll轴分析 ---
subplot(3,4,1);
plot(time_s, target_roll, 'b-', 'LineWidth', 1.5); hold on;
plot(time_s, angle_roll, 'r-', 'LineWidth', 1);
xlabel('时间 (s)'); ylabel('角度 (°)');
title('Roll: 目标 vs 实际');
legend('目标', '实际', 'Location', 'best');
grid on;

subplot(3,4,2);
plot(time_s, err_roll, 'k-', 'LineWidth', 1);
xlabel('时间 (s)'); ylabel('误差 (°)');
title('Roll: 角度误差');
grid on;
yline(0, 'r--');

subplot(3,4,3);
plot(time_s, target_gyro_roll, 'b-', 'LineWidth', 1.5); hold on;
plot(time_s, gyro_x, 'r-', 'LineWidth', 1);
xlabel('时间 (s)'); ylabel('角速度 (rad/s)');
title('Roll: 目标角速度 vs 实际');
legend('目标', '实际', 'Location', 'best');
grid on;

subplot(3,4,4);
plot(time_s, rudder_FR - rudder_FL, 'm-', 'LineWidth', 1);
xlabel('时间 (s)'); ylabel('舵面差动 (°)');
title('Roll: 主翼差动 (FR-FL)');
grid on;

% --- 4.2 Pitch轴分析 ---
subplot(3,4,5);
plot(time_s, target_pitch, 'b-', 'LineWidth', 1.5); hold on;
plot(time_s, angle_pitch, 'r-', 'LineWidth', 1);
xlabel('时间 (s)'); ylabel('角度 (°)');
title('Pitch: 目标 vs 实际');
legend('目标', '实际', 'Location', 'best');
grid on;

subplot(3,4,6);
plot(time_s, err_pitch, 'k-', 'LineWidth', 1);
xlabel('时间 (s)'); ylabel('误差 (°)');
title('Pitch: 角度误差');
grid on;
yline(0, 'r--');

subplot(3,4,7);
plot(time_s, target_gyro_pitch, 'b-', 'LineWidth', 1.5); hold on;
plot(time_s, gyro_y, 'r-', 'LineWidth', 1);
xlabel('时间 (s)'); ylabel('角速度 (rad/s)');
title('Pitch: 目标角速度 vs 实际');
legend('目标', '实际', 'Location', 'best');
grid on;

subplot(3,4,8);
plot(time_s, rudder_BL + rudder_BR, 'm-', 'LineWidth', 1);
xlabel('时间 (s)'); ylabel('舵面同动 (°)');
title('Pitch: V尾同动 (BL+BR)');
grid on;

% --- 4.3 Yaw轴分析 ---
subplot(3,4,9);
plot(time_s, target_yaw, 'b-', 'LineWidth', 1.5); hold on;
plot(time_s, angle_yaw, 'r-', 'LineWidth', 1);
xlabel('时间 (s)'); ylabel('角度 (°)');
title('Yaw: 目标 vs 实际');
legend('目标', '实际', 'Location', 'best');
grid on;

subplot(3,4,10);
plot(time_s, err_yaw, 'k-', 'LineWidth', 1);
xlabel('时间 (s)'); ylabel('误差 (°)');
title('Yaw: 角度误差');
grid on;
yline(0, 'r--');

subplot(3,4,11);
plot(time_s, target_gyro_yaw, 'b-', 'LineWidth', 1.5); hold on;
plot(time_s, gyro_z, 'r-', 'LineWidth', 1);
xlabel('时间 (s)'); ylabel('角速度 (rad/s)');
title('Yaw: 目标角速度 vs 实际');
legend('目标', '实际', 'Location', 'best');
grid on;

subplot(3,4,12);
plot(time_s, rudder_BR - rudder_BL, 'm-', 'LineWidth', 1);
xlabel('时间 (s)'); ylabel('舵面差动 (°)');
title('Yaw: V尾差动 (BR-BL)');
grid on;

%% ==================== 5. 阶跃响应检测与分析 ====================
fprintf('===== 5. 阶跃响应分析 =====\n\n');

% 检测目标角度的阶跃变化
step_threshold = 5;  % 度，认为是阶跃的最小变化
[steps_pitch, step_info_pitch] = detectSteps(target_pitch, time_s, step_threshold);

fprintf('检测到 %d 个Pitch阶跃响应\n', length(steps_pitch));

if ~isempty(steps_pitch)
    figure('Name', 'Pitch阶跃响应详细分析', 'Position', [100, 100, 1200, 400]);
    
    num_steps = min(3, length(steps_pitch));  % 最多显示3个
    for i = 1:num_steps
        idx_start = steps_pitch(i).idx_start;
        idx_end = min(idx_start + round(3/dt), N);  % 3秒窗口
        
        t_window = time_s(idx_start:idx_end) - time_s(idx_start);
        target_window = target_pitch(idx_start:idx_end);
        actual_window = angle_pitch(idx_start:idx_end);
        
        subplot(1, num_steps, i);
        plot(t_window, target_window, 'b--', 'LineWidth', 2); hold on;
        plot(t_window, actual_window, 'r-', 'LineWidth', 1.5);
        xlabel('时间 (s)'); ylabel('Pitch (°)');
        title(sprintf('阶跃 %d: %.1f° → %.1f°', i, steps_pitch(i).from, steps_pitch(i).to));
        legend('目标', '实际');
        grid on;
        
        % 计算响应指标
        [metrics] = analyzeStepResponse(t_window, target_window, actual_window);
        text(0.5, 0.15, sprintf('上升时间: %.2fs\n超调量: %.1f%%\n稳态误差: %.1f°', ...
            metrics.rise_time, metrics.overshoot, metrics.steady_error), ...
            'Units', 'normalized', 'FontSize', 9, 'BackgroundColor', 'w');
    end
end

%% ==================== 6. 内外环带宽分析 ====================
fprintf('\n===== 6. 内外环带宽分析 =====\n\n');

figure('Name', '内外环频率分析', 'Position', [100, 100, 1000, 600]);

% 计算角度和角速度的功率谱
Fs = 1/dt;  % 采样频率

subplot(2,3,1);
[pxx_angle, f_angle] = pwelch(err_roll, [], [], [], Fs);
semilogy(f_angle, pxx_angle);
xlabel('频率 (Hz)'); ylabel('功率谱');
title('Roll角度误差频谱');
grid on;

subplot(2,3,2);
[pxx_angle, f_angle] = pwelch(err_pitch, [], [], [], Fs);
semilogy(f_angle, pxx_angle);
xlabel('频率 (Hz)'); ylabel('功率谱');
title('Pitch角度误差频谱');
grid on;

subplot(2,3,3);
[pxx_angle, f_angle] = pwelch(err_yaw, [], [], [], Fs);
semilogy(f_angle, pxx_angle);
xlabel('频率 (Hz)'); ylabel('功率谱');
title('Yaw角度误差频谱');
grid on;

subplot(2,3,4);
[pxx_gyro, f_gyro] = pwelch(err_gyro_roll, [], [], [], Fs);
semilogy(f_gyro, pxx_gyro);
xlabel('频率 (Hz)'); ylabel('功率谱');
title('Roll角速度误差频谱');
grid on;

subplot(2,3,5);
[pxx_gyro, f_gyro] = pwelch(err_gyro_pitch, [], [], [], Fs);
semilogy(f_gyro, pxx_gyro);
xlabel('频率 (Hz)'); ylabel('功率谱');
title('Pitch角速度误差频谱');
grid on;

subplot(2,3,6);
[pxx_gyro, f_gyro] = pwelch(err_gyro_yaw, [], [], [], Fs);
semilogy(f_gyro, pxx_gyro);
xlabel('频率 (Hz)'); ylabel('功率谱');
title('Yaw角速度误差频谱');
grid on;

%% ==================== 7. PID参数建议 ====================
fprintf('\n===== 7. PID参数调整建议 =====\n\n');

% 统计分析
stats.roll.angle_err_rms = rms(err_roll);
stats.roll.gyro_err_rms = rms(err_gyro_roll);
stats.pitch.angle_err_rms = rms(err_pitch);
stats.pitch.gyro_err_rms = rms(err_gyro_pitch);
stats.yaw.angle_err_rms = rms(err_yaw);
stats.yaw.gyro_err_rms = rms(err_gyro_yaw);

fprintf('当前误差统计 (RMS):\n');
fprintf('  Roll:  角度误差 = %.2f°,  角速度误差 = %.3f rad/s\n', stats.roll.angle_err_rms, stats.roll.gyro_err_rms);
fprintf('  Pitch: 角度误差 = %.2f°,  角速度误差 = %.3f rad/s\n', stats.pitch.angle_err_rms, stats.pitch.gyro_err_rms);
fprintf('  Yaw:   角度误差 = %.2f°,  角速度误差 = %.3f rad/s\n', stats.yaw.angle_err_rms, stats.yaw.gyro_err_rms);

fprintf('\n--- 参数调整建议 ---\n\n');

% Roll轴建议
fprintf('【Roll轴】\n');
if stats.roll.angle_err_rms > 10
    fprintf('  ⚠ 角度误差较大(%.1f°), 建议:\n', stats.roll.angle_err_rms);
    fprintf('    - 外环Kp: 0.1 → 0.15 (提高响应速度)\n');
    new_Kp_roll_outer = 0.15;
else
    fprintf('  ✓ 角度误差可接受(%.1f°)\n', stats.roll.angle_err_rms);
    new_Kp_roll_outer = PID_outer.roll.Kp;
end
if stats.roll.gyro_err_rms > 0.5
    fprintf('  ⚠ 角速度跟踪差(%.2f rad/s), 建议:\n', stats.roll.gyro_err_rms);
    fprintf('    - 内环Kp: 10 → 15 (提高力矩响应)\n');
    new_Kp_roll_inner = 15;
else
    new_Kp_roll_inner = PID_inner.roll.Kp;
end

% Pitch轴建议
fprintf('\n【Pitch轴】\n');
if stats.pitch.angle_err_rms > 8
    fprintf('  ⚠ 角度误差较大(%.1f°), 建议:\n', stats.pitch.angle_err_rms);
    fprintf('    - 外环Kp: 0.1 → 0.12\n');
    new_Kp_pitch_outer = 0.12;
else
    fprintf('  ✓ 角度误差可接受(%.1f°)\n', stats.pitch.angle_err_rms);
    new_Kp_pitch_outer = PID_outer.pitch.Kp;
end

% Yaw轴建议
fprintf('\n【Yaw轴】\n');
if stats.yaw.angle_err_rms > 15
    fprintf('  ⚠ Yaw控制较弱(误差%.1f°), 这是正常的因为V尾yaw效率低\n', stats.yaw.angle_err_rms);
    fprintf('    - 可考虑增加内环Kp: 10 → 12\n');
end

%% ==================== 8. 生成建议的C代码 ====================
fprintf('\n\n===== 8. 建议的PID参数 (C代码) =====\n\n');

fprintf('// ===== 基于数据分析的PID参数建议 =====\n');
fprintf('// 外环 (角度环)\n');
fprintf('Contrl_data.Pose_PID[AXIS_ROLL]  = PID_init(%.2f, 0, 0, 0, 1.5, -1.5);\n', new_Kp_roll_outer);
fprintf('Contrl_data.Pose_PID[AXIS_PITCH] = PID_init(%.2f, 0, 0, 0, 2.0, -2.0);\n', new_Kp_pitch_outer);
fprintf('Contrl_data.Pose_PID[AXIS_YAW]   = PID_init(0.10, 0, 0, 0, 2.0, -2.0);\n');
fprintf('\n// 内环 (角速度环)\n');
fprintf('Contrl_data.torque_PID[AXIS_ROLL]  = PID_init(%.0f, 0, 0, 50, 60, -60);\n', new_Kp_roll_inner);
fprintf('Contrl_data.torque_PID[AXIS_PITCH] = PID_init(10, 0, 0, 200, 60, -60);\n');
fprintf('Contrl_data.torque_PID[AXIS_YAW]   = PID_init(10, 0, 0, 50, 60, -60);\n');

%% ==================== 9. 耦合分析 ====================
fprintf('\n\n===== 9. 轴间耦合分析 =====\n\n');

figure('Name', '轴间耦合分析', 'Position', [100, 100, 800, 600]);

% Roll-Pitch耦合
subplot(2,2,1);
scatter(err_roll, err_pitch, 1, 'b', 'filled', 'MarkerFaceAlpha', 0.3);
xlabel('Roll误差 (°)'); ylabel('Pitch误差 (°)');
title('Roll-Pitch耦合');
grid on;
corr_rp = corrcoef(err_roll, err_pitch);
text(0.05, 0.9, sprintf('相关系数: %.3f', corr_rp(1,2)), 'Units', 'normalized');

% Roll-Yaw耦合
subplot(2,2,2);
scatter(err_roll, err_yaw, 1, 'r', 'filled', 'MarkerFaceAlpha', 0.3);
xlabel('Roll误差 (°)'); ylabel('Yaw误差 (°)');
title('Roll-Yaw耦合');
grid on;
corr_ry = corrcoef(err_roll, err_yaw);
text(0.05, 0.9, sprintf('相关系数: %.3f', corr_ry(1,2)), 'Units', 'normalized');

% Pitch-Yaw耦合
subplot(2,2,3);
scatter(err_pitch, err_yaw, 1, 'g', 'filled', 'MarkerFaceAlpha', 0.3);
xlabel('Pitch误差 (°)'); ylabel('Yaw误差 (°)');
title('Pitch-Yaw耦合');
grid on;
corr_py = corrcoef(err_pitch, err_yaw);
text(0.05, 0.9, sprintf('相关系数: %.3f', corr_py(1,2)), 'Units', 'normalized');

% 舵面相关性
subplot(2,2,4);
rudder_corr = corrcoef([rudder_FR, rudder_FL, rudder_BL, rudder_BR]);
imagesc(rudder_corr);
colorbar;
set(gca, 'XTick', 1:4, 'XTickLabel', {'FR', 'FL', 'BL', 'BR'});
set(gca, 'YTick', 1:4, 'YTickLabel', {'FR', 'FL', 'BL', 'BR'});
title('舵面相关性矩阵');
colormap('jet');

fprintf('耦合分析:\n');
fprintf('  Roll-Pitch相关: %.3f %s\n', corr_rp(1,2), getCouplingLevel(corr_rp(1,2)));
fprintf('  Roll-Yaw相关:   %.3f %s\n', corr_ry(1,2), getCouplingLevel(corr_ry(1,2)));
fprintf('  Pitch-Yaw相关:  %.3f %s\n', corr_py(1,2), getCouplingLevel(corr_py(1,2)));

fprintf('\n===== 分析完成 =====\n');

%% ==================== 辅助函数 ====================

function [steps, step_info] = detectSteps(signal, time, threshold)
    % 检测信号中的阶跃变化
    diff_signal = diff(signal);
    step_idx = find(abs(diff_signal) > threshold);
    
    steps = [];
    step_info = [];
    
    min_gap = 50;  % 最小间隔采样点
    last_idx = -min_gap;
    
    for i = 1:length(step_idx)
        if step_idx(i) - last_idx > min_gap
            s.idx_start = step_idx(i);
            s.from = signal(step_idx(i));
            s.to = signal(min(step_idx(i)+1, length(signal)));
            s.time = time(step_idx(i));
            steps = [steps, s];
            last_idx = step_idx(i);
        end
    end
end

function metrics = analyzeStepResponse(t, target, actual)
    % 分析阶跃响应指标
    metrics = struct();
    
    % 目标值变化
    target_start = target(1);
    target_end = target(end);
    step_size = target_end - target_start;
    
    if abs(step_size) < 1
        metrics.rise_time = NaN;
        metrics.overshoot = 0;
        metrics.steady_error = mean(target(end-10:end) - actual(end-10:end));
        return;
    end
    
    % 上升时间 (10%到90%)
    threshold_10 = target_start + 0.1 * step_size;
    threshold_90 = target_start + 0.9 * step_size;
    
    if step_size > 0
        idx_10 = find(actual >= threshold_10, 1, 'first');
        idx_90 = find(actual >= threshold_90, 1, 'first');
    else
        idx_10 = find(actual <= threshold_10, 1, 'first');
        idx_90 = find(actual <= threshold_90, 1, 'first');
    end
    
    if ~isempty(idx_10) && ~isempty(idx_90) && idx_90 > idx_10
        metrics.rise_time = t(idx_90) - t(idx_10);
    else
        metrics.rise_time = NaN;
    end
    
    % 超调量
    if step_size > 0
        peak = max(actual);
        metrics.overshoot = max(0, (peak - target_end) / abs(step_size) * 100);
    else
        peak = min(actual);
        metrics.overshoot = max(0, (target_end - peak) / abs(step_size) * 100);
    end
    
    % 稳态误差 (最后10个点)
    n_steady = min(10, length(t));
    metrics.steady_error = mean(target(end-n_steady+1:end) - actual(end-n_steady+1:end));
end

function level = getCouplingLevel(corr)
    % 判断耦合程度
    corr = abs(corr);
    if corr < 0.3
        level = '(弱耦合 ✓)';
    elseif corr < 0.6
        level = '(中等耦合)';
    else
        level = '(强耦合 ⚠)';
    end
end

function y = wrapTo180(x)
    % 将角度限制在-180到180度
    y = mod(x + 180, 360) - 180;
end