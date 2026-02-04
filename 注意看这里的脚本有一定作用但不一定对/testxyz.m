%% 轴向对齐扫描工具：到底哪个是Roll？
clc; clear; close all;

% --- 1. 选择并读取文件 ---
[file, path] = uigetfile('*.csv', '选择飞行数据');
if isequal(file,0), return; end
T_raw = readtable(fullfile(path, file));
T_raw = sortrows(T_raw, 'packet_timestamp');

% --- 2. 自动列名匹配引擎 ---
all_cols = T_raw.Properties.VariableNames;
function data = force_get(hint, cols, tab)
    idx = find(contains(lower(cols), lower(hint)));
    if isempty(idx)
        [s, ~] = listdlg('PromptString', ['找不到 ', hint, '，请手动选：'], 'ListString', cols);
        data = tab{:, s};
    else
        data = tab{:, idx(1)};
    end
end

r1 = force_get('rudder1', all_cols, T_raw);
r2 = force_get('rudder2', all_cols, T_raw);
gx = force_get('gyro_x', all_cols, T_raw);
gy = force_get('gyro_y', all_cols, T_raw);
gz = force_get('gyro_z', all_cols, T_raw);
ts = force_get('packet_timestamp', all_cols, T_raw);

% --- 3. 计算采样率与指令 ---
dt_vec = diff(ts);
dt = median(dt_vec);
if dt > 1, dt = dt/1000; end % ms转s
fs = 1/dt;

% 你的副翼主指令 (如果结果全为负，运行完后手动改为 r2-r1 再跑一次)
cmd = r1 - r2; 

% 计算三个轴的角加速度 (响应)
ax = [0; diff(gx)] / dt;
ay = [0; diff(gy)] / dt;
az = [0; diff(gz)] / dt;

% 适当平滑减少高频噪点
ax = movmean(ax, 10); ay = movmean(ay, 10); az = movmean(az, 10);

% --- 4. 互相关扫描 (寻找各轴延迟与强度) ---
max_l = 100; % 搜索100个采样点内的延迟
[rx, lx] = xcorr(ax - mean(ax), cmd - mean(cmd), max_l, 'coeff');
[ry, ly] = xcorr(ay - mean(ay), cmd - mean(cmd), max_l, 'coeff');
[rz, lz] = xcorr(az - mean(az), cmd - mean(cmd), max_l, 'coeff');

% --- 5. 绘图对比 ---
figure('Color', 'w', 'Name', '轴向扫描诊断', 'Position', [100 100 1000 800]);

subplot(3,1,1); plot(lx*dt*1000, rx, 'LineWidth', 1.5); title('与 X 轴(Roll?) 相关性'); grid on; ylabel('R');
subplot(3,1,2); plot(ly*dt*1000, ry, 'LineWidth', 1.5); title('与 Y 轴(Pitch?) 相关性'); grid on; ylabel('R');
subplot(3,1,3); plot(lz*dt*1000, rz, 'LineWidth', 1.5); title('与 Z 轴(Yaw?) 相关性'); grid on; ylabel('R');
xlabel('时间延迟 (ms)');

% --- 6. 自动分析建议 ---
[mx, ix] = max(abs(rx)); [my, iy] = max(abs(ry)); [mz, iz] = max(abs(rz));
[best_val, best_axis] = max([mx, my, mz]);
ax_names = {'X', 'Y', 'Z'};
corr_list = [rx(ix), ry(iy), rz(iz)];
lag_list = [lx(ix), ly(iy), lz(iz)];

fprintf('\n========= 扫描报告 =========\n');
fprintf('最强响应轴: 【%s 轴】\n', ax_names{best_axis});
fprintf('最大相关系数: %.4f\n', corr_list(best_axis));
fprintf('物理估算延迟: %.1f ms\n', abs(lag_list(best_axis))*dt*1000);

if abs(corr_list(best_axis)) < 0.3
    fprintf('结论：信号太弱！可能原因：1. 截取数据没动作；2. 舵机1/2不是副翼。\n');
elseif corr_list(best_axis) < 0
    fprintf('结论：【极性反向】！飞控输出正指令，飞机产生负加速度。需反转输出或IMU定义。\n');
else
    fprintf('结论：【极性正确】。\n');
end

if best_axis ~= 1
    fprintf('警告：你的 Roll 指令响应在 %s 轴上！坐标系映射可能有误。\n', ax_names{best_axis});
end