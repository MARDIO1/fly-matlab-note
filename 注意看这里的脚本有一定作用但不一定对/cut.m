%% 飞控全维度数据分析与手动截取工具 (兼容版)
clc; clear; close all;

% 1. 选择并读取文件
[file, path] = uigetfile('*.csv', '选择飞控导出文件');
if isequal(file,0), return; end
fullPath = fullfile(path, file);

fprintf('正在加载并清洗数据...\n');
opts = detectImportOptions(fullPath);
T = readtable(fullPath, opts);

%% 2. 核心清洗：排序、去重、断裂处理
T = sortrows(T, 'packet_timestamp');
[~, unique_indices] = unique(T.packet_timestamp, 'first'); 
T_clean = T(unique_indices, :);

% 检测断裂点并插入 NaN (解决跳点连线问题)
time_raw = T_clean.packet_timestamp;
dt = diff(time_raw);
median_dt = median(dt);
gap_indices = find(dt > (median_dt * 2.5));

if ~isempty(gap_indices)
    nan_row = T_clean(1, :);
    for col = 1:width(nan_row)
        if isnumeric(nan_row{1,col})
            nan_row{1,col} = NaN;
        else
            nan_row{1,col} = {missing};
        end
    end
    for i = length(gap_indices):-1:1
        idx = gap_indices(i);
        row_to_insert = nan_row;
        row_to_insert.packet_timestamp = time_raw(idx) + median_dt/2;
        T_clean = [T_clean(1:idx, :); row_to_insert; T_clean(idx+1:end, :)];
    end
end

time_axis = T_clean.packet_timestamp;

%% 3. 绘制交互图表
fig = figure('Color', 'w', 'Name', ['手动截取: ', file], 'Position', [50 50 1200 900]);
color_set = ["#0072BD", "#D95319", "#EDB120", "#7E2F8E"];

% 定义字段 (请确保你的CSV列名完全一致)
fields = {{'angle_roll','angle_pitch','angle_yaw'}, ...
          {'gyro_x','gyro_y','gyro_z'}, ...
          {'acc_x','acc_y','acc_z'}, ...
          {'rudder1','rudder2','rudder3','rudder4'}};
ylabels = {'姿态角 (deg)', '角速度 (deg/s)', '加速度 (mg)', '舵面输出'};

axs = zeros(4,1);
for i = 1:4
    axs(i) = subplot(4,1,i);
    hold on;
    current_fields = fields{i};
    for j = 1:length(current_fields)
        fname = current_fields{j};
        if ismember(fname, T_clean.Properties.VariableNames)
            plot(time_axis, T_clean.(fname), 'LineWidth', 1, 'Color', color_set(mod(j-1,4)+1), 'DisplayName', fname);
        end
    end
    ylabel(ylabels{i});
    grid on; box on;
    if i < 4, set(gca, 'XTickLabel', []); end
end
xlabel('时间戳 (Timestamp)');
linkaxes(axs, 'x');

%% 4. 交互截取逻辑
fprintf('\n--- 操作指南 ---\n');
fprintf('1. 使用工具栏放大镜观察波形细节。\n');
fprintf('2. 确认位置后，在鼠标处于图表区域时，按键盘上的 [回车 Enter]。\n');
fprintf('3. 接着在图上依次点击两个位置（起点和终点）。\n');

pause; % 等待回车

% 点选起点
fprintf('请点击起点... \n');
[x1, ~] = ginput(1);
for i = 1:4
    axes(axs(i));
    yl = get(gca, 'YLim');
    line([x1 x1], yl, 'Color', 'g', 'LineWidth', 2, 'LineStyle', '--');
    text(x1, yl(2), ' 起点', 'Color', 'g', 'VerticalAlignment', 'top');
end

% 点选终点
fprintf('请点击终点... \n');
[x2, ~] = ginput(1);
for i = 1:4
    axes(axs(i));
    yl = get(gca, 'YLim');
    line([x2 x2], yl, 'Color', 'r', 'LineWidth', 2, 'LineStyle', '--');
    text(x2, yl(2), ' 终点', 'Color', 'r', 'VerticalAlignment', 'top');
end

% 计算区间
t_start = min(x1, x2);
t_end = max(x1, x2);

% 5. 导出
final_mask = T_clean.packet_timestamp >= t_start & T_clean.packet_timestamp <= t_end & ~isnan(T_clean.acc_y);
T_flight = T_clean(final_mask, :);

if isempty(T_flight)
    fprintf('错误：所选区间内没有有效数据！\n');
else
    [~, fname_only, fext] = fileparts(file);
    outputName = fullfile(path, [fname_only, '_flight', fext]);
    writetable(T_flight, outputName);

    fprintf('\n截取成功！\n');
    fprintf('时间区间: %.3f 到 %.3f\n', t_start, t_end);
    fprintf('文件已保存: %s\n', outputName);
end