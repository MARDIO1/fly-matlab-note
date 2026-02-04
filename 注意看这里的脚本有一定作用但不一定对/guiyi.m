%% 固定翼主翼 Roll 轴动力学深度分析工具 (V2.0)
clc; clear; close all;

% --- 1. 加载与清洗 ---
[file, path] = uigetfile('*.csv', '选择飞行数据');
if isequal(file,0), return; end
T = readtable(fullfile(path, file));
T = sortrows(T, 'packet_timestamp');
[~, uIdx] = unique(T.packet_timestamp, 'first');
T = T(uIdx, :);

% --- 2. 信号提取与预处理 ---
fs = 100; dt = 1/fs;
t_raw = (T.packet_timestamp - T.packet_timestamp(1));

% 主翼差动指令 (假设 1:右前, 2:左前)
cmd_roll = T.rudder1 - T.rudder2; 

% 反馈信号处理
gyro_x = T.gyro_x;
% 计算角加速度并加强平滑，减少求导噪声带来的"低相关性"假象
alpha_x = [0; diff(gyro_x)] / dt;
alpha_x = movmean(alpha_x, 8); % 增加平滑窗口到8点

% --- 3. 核心算法：相关性与极性自动匹配 ---
% 我们对比：舵机指令 vs 角加速度 (这是物理受力最直接的关系)
u = cmd_roll - mean(cmd_roll, 'omitnan');
y = alpha_x - mean(alpha_x, 'omitnan');

% 搜索正负 1 秒内的延迟
[corr_val, lags] = xcorr(y, u, 100, 'coeff');

% 【核心改进】寻找绝对值最大的相关点（处理正负反向问题）
[~, max_idx] = max(abs(corr_val)); 
max_corr = corr_val(max_idx);       % 保留原始正负号
delay_samples = lags(max_idx); 
delay_ms = delay_samples * dt * 1000;

% --- 4. 生成对齐信号 (线性平移) ---
y_aligned = NaN(size(y));
if delay_samples > 0
    y_aligned(1:end-delay_samples) = y(delay_samples+1:end);
elseif delay_samples < 0
    y_aligned(-delay_samples+1:end) = y(1:end+delay_samples);
else
    y_aligned = y;
end

% 如果相关性是负的，说明控制逻辑是反向的，为了观察方便，我们在绘图时反转它
if max_corr < 0
    y_norm_display = -y_aligned / (max(abs(y_aligned)) + 1e-6);
    polarity_str = '反向响应 (已自动翻转观察)';
else
    y_norm_display = y_aligned / (max(abs(y_aligned)) + 1e-6);
    polarity_str = '正向响应';
end
u_norm_display = u / (max(abs(u)) + 1e-6);

% --- 5. 绘图展示 ---
figure('Color', 'w', 'Name', 'Roll轴深度分析 V2.0', 'Position', [100, 100, 1000, 700]);

% 子图1：物理量直接对比
subplot(2,1,1);
yyaxis left;
plot(t_raw, cmd_roll, 'LineWidth', 1); ylabel('主翼差动指令 (R1-R2)');
yyaxis right;
plot(t_raw, gyro_x, 'LineWidth', 1); ylabel('滚转角速度 (deg/s)');
title(['原始响应时序 (延迟: ', num2str(delay_ms, '%.1f'), ' ms)']);
grid on;

% 子图2：对齐后的特征匹配
subplot(2,1,2);
plot(t_raw, u_norm_display, 'LineWidth', 1.5, 'DisplayName', '舵机指令 (归一化)');
hold on;
plot(t_raw, y_norm_display, 'LineWidth', 1.5, 'DisplayName', '角加速度 (平移对齐后)');
title(['特征匹配度 (R = ', num2str(abs(max_corr), '%.4f'), ' | ', polarity_str, ')']);
xlabel('时间 (s)'); ylabel('归一化幅值');
legend('Location', 'best');
grid on;

% --- 6. 诊断结论 ---
fprintf('\n========= 诊断报告 =========\n');
fprintf('1. 识别极性: %s\n', polarity_str);
fprintf('2. 匹配程度 (R): %.4f ', abs(max_corr));
if abs(max_corr) > 0.6, fprintf('(优秀)\n'); else, fprintf('(较低，建议检查数据选段)\n'); end
fprintf('3. 控制延迟: %.1f ms\n', delay_ms);
if delay_ms < 0, fprintf('   *警告：检测到指令滞后于响应，说明飞控输出的是"补偿量"而非"驱动量"。\n'); end
fprintf('============================\n');