%% 固定翼 Roll 轴贡献度拆解 (严格对应：1,2为前主翼 | 4,3为后尾翼)
clc; clear; close all;

% --- 1. 加载数据 ---
[file, path] = uigetfile('*.csv', '选择空中纯净段数据');
if isequal(file,0), return; end
T = readtable(fullfile(path, file));
T = sortrows(T, 'packet_timestamp');

% --- 2. 严格物理定义提取 ---
fs = 100; dt = 1/fs;
t_raw = (T.packet_timestamp - T.packet_timestamp(1));

% 【核心定义区】
% 主翼：1右前，2左前 -> 差动产生Roll
cmd_main_wing = T.rudder1 - T.rudder2; 

% 后翼：4右后，3左后 -> 差动产生Roll (十字尾翼或V尾的滚转分量)
cmd_tail_wing = T.rudder4 - T.rudder3; 

% 响应：滚转角速度 -> 转化为角加速度
gyro_x = T.gyro_x;
alpha_x = [0; diff(gyro_x)] / dt;
alpha_x = movmean(alpha_x, 8); % 降噪

% --- 3. 信号对齐 ---
% 以主翼指令为基准，寻找响应的延迟
[c, lags] = xcorr(alpha_x, cmd_main_wing, 50, 'coeff');
[~, max_idx] = max(abs(c));
best_lag = lags(max_idx);
alpha_aligned = circshift(alpha_x, -best_lag);

% --- 4. 最小二乘法回归 ---
% 目的：找出 alpha_x = K1*主翼 + K2*后翼 + Constant
% 剔除对齐边缘
valid = (abs(best_lag)+1) : (length(alpha_aligned)-abs(best_lag));
Y = alpha_aligned(valid);
X1 = cmd_main_wing(valid);
X2 = cmd_tail_wing(valid);

% 归一化自变量以便直接比较"威力" (贡献度)
X1_n = (X1 - mean(X1)) / std(X1);
X2_n = (X2 - mean(X2)) / std(X2);
Y_n = (Y - mean(Y));

% 执行线性回归
coeffs = [X1_n, X2_n] \ Y_n; 
K_main = coeffs(1);
K_tail = coeffs(2);

% --- 5. 结果计算与展示 ---
main_power = abs(K_main);
tail_power = abs(K_tail);
total_power = main_power + tail_power;

figure('Color', 'w', 'Name', '前主翼 vs 后尾翼 Roll轴效能拆解');

% 子图1：时域波形对齐查看
subplot(2,1,1);
plot(t_raw(valid), Y_n/max(abs(Y_n)), 'k', 'LineWidth', 1.5, 'DisplayName', '实际响应(Alpha X)');
hold on;
plot(t_raw(valid), X1_n * K_main / max(abs(Y_n)), 'r', 'DisplayName', '主翼贡献');
plot(t_raw(valid), X2_n * K_tail / max(abs(Y_n)), 'b', 'DisplayName', '后翼贡献');
title(['对齐后的Roll轴推力来源拆解 (延迟: ', num2str(best_lag*10), 'ms)']);
ylabel('归一化影响'); legend; grid on;

% 子图2：贡献度比例
subplot(2,1,2);
bar([main_power/total_power*100, tail_power/total_power*100]);
set(gca, 'XTickLabel', {'前主翼 (R1-R2)', '后尾翼 (R4-R3)'});
ylabel('贡献比例 (%)');
title(['物理舵效权重分析 (主翼: ', num2str(main_power/total_power*100, '%.1f'), '%)']);
grid on;

% --- 诊断输出 ---
fprintf('\n========= 物理特征诊断 =========\n');
fprintf('1. 前主翼响应系数: %.4f\n', K_main);
fprintf('2. 后尾翼响应系数: %.4f\n', K_tail);
if (K_main * K_tail < 0)
    fprintf('【警告】检测到主翼和后翼的滚转极性相反！它们在互相抵消力矩。\n');
end
fprintf('3. 主翼贡献度: %.1f%%\n', (main_power/total_power)*100);
fprintf('================================\n');