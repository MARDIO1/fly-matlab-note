%% 三轴相关性与时延诊断看板
fs = 1/mean(diff(time_s_subset));
dt = 1/fs;

% 1. 信号准备：对角速度进行低通滤波以平滑加速度（截止频率 10Hz）
cutoff = 10;
gx_f = lowpass(data_subset.gyro_x, cutoff, fs);
gy_f = lowpass(data_subset.gyro_y, cutoff, fs);
gz_f = lowpass(data_subset.gyro_z, cutoff, fs);

% 2. 计算角加速度
ax = diff(gx_f) / dt;
ay = diff(gy_f) / dt;
az = diff(gz_f) / dt;

% 3. 绘图
figure('Color', 'w', 'Name', '三轴舵效相关性看板', 'Position', [100, 100, 1000, 800]);

% --- ROLL 轴诊断 ---
subplot(3,1,1);
yyaxis left;
% 滚转主力通常是 FR - FL (差动)
plot(data_subset.surface_FR - data_subset.surface_FL, 'LineWidth', 1.2, 'DisplayName', 'Front Differential (FR-FL)');
ylabel('Roll 指令 (deg)');
hold on;
plot(data_subset.surface_BL - data_subset.surface_BR, '--', 'LineWidth', 1.0, 'DisplayName', 'Back Differential (BL-BR)');
yyaxis right;
plot(ax, 'Color', [1 0 0 0.5], 'LineWidth', 1, 'DisplayName', 'Roll Accel (\alpha_x)');
ylabel('rad/s^2');
title('Roll 轴：前/后差动指令 vs 滚转加速度');
legend('Location', 'northeast', 'FontSize', 8);
grid on;

% --- PITCH 轴诊断 ---
subplot(3,1,2);
yyaxis left;
% 俯仰通常是同向动作 (BL + BR) / 2
plot((data_subset.surface_BL + data_subset.surface_BR)/2, 'LineWidth', 1.2, 'DisplayName', 'Back Sum (BL+BR)');
ylabel('Pitch 指令 (deg)');
hold on;
plot((data_subset.surface_FR + data_subset.surface_FL)/2, '--', 'LineWidth', 1.0, 'DisplayName', 'Front Sum (FR+FL)');
yyaxis right;
plot(ay, 'Color', [0 0.7 0 0.5], 'LineWidth', 1, 'DisplayName', 'Pitch Accel (\alpha_y)');
ylabel('rad/s^2');
title('Pitch 轴：前/后同向指令 vs 俯仰加速度');
legend('Location', 'northeast', 'FontSize', 8);
grid on;

% --- YAW 轴诊断 ---
subplot(3,1,3);
yyaxis left;
% 偏航通常靠尾部差动或特定组合
plot(data_subset.surface_BL - data_subset.surface_BR, 'LineWidth', 1.2, 'DisplayName', 'Back Diff (BL-BR)');
ylabel('Yaw 指令 (deg)');
yyaxis right;
plot(az, 'Color', [0 0 1 0.5], 'LineWidth', 1, 'DisplayName', 'Yaw Accel (\alpha_z)');
ylabel('rad/s^2');
title('Yaw 轴：尾部差动 vs 偏航加速度');
legend('Location', 'northeast', 'FontSize', 8);
grid on;

xlabel('采样点');

%% 自动计算并显示三轴最佳对齐延迟 (Cross-Correlation)
[xc_r, lags_r] = xcorr(ax, data_subset.surface_FR(1:end-1) - data_subset.surface_FL(1:end-1), 20);
[~, I_r] = max(abs(xc_r));
fprintf('建议：Roll 轴角加速度滞后指令约 %d 个采样点\n', abs(lags_r(I_r)));

[xc_p, lags_p] = xcorr(ay, (data_subset.surface_BL(1:end-1) + data_subset.surface_BR(1:end-1))/2, 20);
[~, I_p] = max(abs(xc_p));
fprintf('建议：Pitch 轴角加速度滞后指令约 %d 个采样点\n', abs(lags_p(I_p)));