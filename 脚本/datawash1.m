%% 1. 数据加载与预处理 (保持你原有的逻辑)
data = b1; 
[~, uniqueIdx] = unique(data.timestamp, 'first');
data = data(uniqueIdx, :);
data{:, 2:end} = fillmissing(data{:, 2:end}, 'linear');
time_s = seconds(data.timestamp - data.timestamp(1));

%% 2. 按照需求进行数据截取与处理

% --- 需求1：截取 1-5 秒数据 ---
% 找到时间在 1s 到 5s 之间的行
idx = (time_s >= 1 & time_s <= 5);
data_subset = data(idx, :);

% --- 需求2：删除全为 0 的 target 相关列 ---
target_cols = ["target_roll", "target_pitch", "target_yaw", "target_wx", "target_wy", "target_wz"];
% 检查 data_subset 中这些列是否全为 0
is_all_zero = varfun(@(x) all(x == 0), data_subset, 'InputVariables', target_cols);
cols_to_remove = target_cols(logical(table2array(is_all_zero)));
data_subset(:, cols_to_remove) = [];

data_subset.surface_FR = 5 - data_subset.rudder1;
data_subset.surface_FL = data_subset.rudder2 + 5;
data_subset.surface_BL = data_subset.rudder3;
data_subset.surface_BR = -6 - data_subset.rudder4;

%% 3. 保存到 CSV 文件 (需求4)
output_name = 'processed_flight_data.csv';
writetable(data_subset, output_name);

fprintf('处理完成！\n1. 已截取1-5秒数据。\n2. 已删除全零目标列: %s\n3. 已完成Surface逆映射计算。\n4. 文件已保存至: %s\n', ...
    strjoin(cols_to_remove, ', '), output_name);

%% 4. 验证性可视化 (可选)
figure('Name', '逆映射结果检查', 'Color', 'w');
subplot(2,1,1);
plot(data_subset.rudder1); hold on; plot(data_subset.rudder2);
legend('Rudder FR (Raw)', 'Rudder FL (Raw)'); title('原始舵机数据');
subplot(2,1,2);
plot(data_subset.surface_FR); hold on; plot(data_subset.surface_FL);
legend('Surface FR (Mapped)', 'Surface FL (Mapped)'); title('逆映射后的翼面角度');