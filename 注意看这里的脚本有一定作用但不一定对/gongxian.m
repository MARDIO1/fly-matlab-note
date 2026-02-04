%% 全自动适配：飞控物理特性深度诊断工具 (基础版兼容)
clc; clear; close all;

% --- 1. 智能文件读取 ---
[file, path] = uigetfile('*.csv', '选择任意飞控导出文件');
if isequal(file,0), return; end
T = readtable(fullfile(path, file), 'VariableNamingRule', 'preserve');

% 获取所有列名
cols = T.Properties.VariableNames;

% --- 2. 提取信号 (内部函数定义) ---
helper_find = @(k) helper_find_col(T, cols, k);

r1 = helper_find({'rudder1', 'servo1', 'pwm1', 'out1'});
r2 = helper_find({'rudder2', 'servo2', 'pwm2', 'out2'});
r3 = helper_find({'rudder3', 'servo3', 'pwm3', 'out3'});
r4 = helper_find({'rudder4', 'servo4', 'pwm4', 'out4'});

gx = helper_find({'gyro_x', 'gyrox', 'ratest_x'});
gy = helper_find({'gyro_y', 'gyroy', 'ratest_y'});
gz = helper_find({'gyro_z', 'gyroz', 'ratest_z'});
ts = helper_find({'timestamp', 'time', 'packet'});

% --- 3. 自动时间戳对齐 ---
[~, time_idx] = max(contains(lower(cols), 'timestamp') | contains(lower(cols), 'time'));
T = sortrows(T, time_idx);
dt = median(diff(ts));
if dt > 1, dt = dt / 1000; end 

% --- 4. 自动轴向探测 ---
cmd_main = (r1 - mean(r1)) - (r2 - mean(r2));
cmd_tail = (r3 - mean(r3)) - (r4 - mean(r4));
total_cmd = cmd_main + cmd_tail;

% 计算角加速度趋势
ax = [0; diff(gx)]/dt; ay = [0; diff(gy)]/dt; az = [0; diff(gz)]/dt;
ax = movmean(ax, 10); ay = movmean(ay, 10); az = movmean(az, 10);

% 互相关扫描 (寻找各轴相关性)
[rx, lx] = xcorr(ax - mean(ax), total_cmd, 50, 'coeff');
[ry, ly] = xcorr(ay - mean(ay), total_cmd, 50, 'coeff');
[rz, lz] = xcorr(az - mean(az), total_cmd, 50, 'coeff');

[val_x, ix] = max(abs(rx)); [val_y, iy] = max(abs(ry)); [val_z, iz] = max(abs(rz));
[~, best_axis_idx] = max([val_x, val_y, val_z]);

resps = {ax, ay, az};
ax_names = {'X轴', 'Y轴', 'Z轴'};
alpha_target = resps{best_axis_idx}; 
final_r_vals = [rx(ix), ry(iy), rz(iz)];
final_r = final_r_vals(best_axis_idx);
final_lags = [lx(ix), ly(iy), lz(iz)];
delay_ms = final_lags(best_axis_idx) * dt * 1000;

% --- 5. 计算共线性 R (使用基础函数) ---
C = corrcoef(cmd_main, cmd_tail);
R_collinear = C(1,2);

% --- 6. 绘图展示 ---
figure('Color', 'w', 'Name', ['诊断结果 - 目标轴: ', ax_names{best_axis_idx}], 'Position', [100 100 1100 500]);

% 左图：共线性散点图
subplot(1,2,1);
scatter(cmd_main, cmd_tail, 15, 'filled', 'MarkerFaceAlpha', 0.4); hold on;
% 手动绘制拟合线代替 lsline
if length(cmd_main) > 1
    p = polyfit(cmd_main, cmd_tail, 1);
    px = [min(cmd_main), max(cmd_main)];
    py = polyval(p, px);
    plot(px, py, 'r', 'LineWidth', 2);
end
grid on; xlabel('主翼指令 (r1-r2)'); ylabel('后翼指令 (r3-r4)');
title(['执行器共线性 R = ', num2str(R_collinear, '%.4f')]);

% 右图：响应波形对齐
subplot(1,2,2);
t_axis = (ts - ts(1));
norm_alpha = (alpha_target - mean(alpha_target))/std(alpha_target);
norm_cmd = (total_cmd - mean(total_cmd))/std(total_cmd);
plot(t_axis, norm_alpha, 'k', 'DisplayName', ['实际加速度(', ax_names{best_axis_idx}, ')']); hold on;
plot(t_axis - (delay_ms/1000), norm_cmd, 'r--', 'DisplayName', '对齐后指令');
grid on; xlim([t_axis(round(end/3)), t_axis(round(end/3))+4]);
title(['动力学响应 (延迟: ', num2str(abs(delay_ms), '%.1f'), 'ms, R = ', num2str(final_r, '%.4f'), ')']);
legend;

% --- 7. 诊断报告 ---
fprintf('\n================== 自动诊断报告 ==================\n');
fprintf('1. 轴向探测：副翼指令驱动了飞机的 【%s】\n', ax_names{best_axis_idx});
if best_axis_idx ~= 1, fprintf('   【警告】轴向映射错误！Roll指令本应驱动X轴。\n'); end
fprintf('2. 执行器共线性 R = %.4f\n', R_collinear);
if R_collinear < -0.5, fprintf('   【严重】主后翼在物理上"打架"（抵消）。\n'); end
fprintf('3. 系统极性：%.4f\n', final_r);
if final_r > 0, fprintf('   结论：极性正确。\n'); else, fprintf('   结论：【危险】极性反向！\n'); end
fprintf('4. 链路延迟：%.1f ms\n', abs(delay_ms));

%% --- 辅助函数 ---
function data = helper_find_col(tab, cols, keywords)
    data = [];
    % 尝试精确匹配
    for k = 1:length(keywords)
        idx = find(strcmpi(cols, keywords{k}));
        if ~isempty(idx), data = tab{:, idx(1)}; return; end
    end
    % 尝试模糊包含
    for k = 1:length(keywords)
        idx = find(contains(lower(cols), lower(keywords{k})));
        if ~isempty(idx), data = tab{:, idx(1)}; return; end
    end
    % 手动选
    [s, ok] = listdlg('PromptString', ['请选择 [', keywords{1}, '] 对应的列:'], 'ListString', cols);
    if ok, data = tab{:, s}; else, error('未选择列'); end
end