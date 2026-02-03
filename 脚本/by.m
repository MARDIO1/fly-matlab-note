%% =====================================================================
%  雅可比矩阵拟合 - 适配您的控制系统
%  输出: roll_d, pitch_d, yaw_d (角度指令，单位：度或弧度)
%% =====================================================================

clear; clc; close all;

%% 1. 加载数据
[file, path] = uigetfile('*.csv', '选择黑匣子CSV文件');
if file == 0
    error('未选择文件');
end

data = readtable(fullfile(path, file), 'VariableNamingRule', 'preserve');
fprintf('加载: %d 行 x %d 列\n', height(data), width(data));

%% 2. 定义列名 - 根据您的实际CSV列名修改
% 输入：4个舵面
inputCols = {'rudder1', 'rudder2', 'rudder3', 'rudder4'};

% 输出：姿态指令（对应您代码中的 roll_d, pitch_d, yaw_d）
outputCols = {'angle_roll', 'angle_pitch', 'angle_yaw'};

%% 3. 提取数据
X = zeros(height(data), 4);
Y = zeros(height(data), 3);

for i = 1:4
    if ismember(inputCols{i}, data.Properties.VariableNames)
        X(:,i) = data.(inputCols{i});
    else
        error('找不到列: %s', inputCols{i});
    end
end

for i = 1:3
    if ismember(outputCols{i}, data.Properties.VariableNames)
        Y(:,i) = data.(outputCols{i});
    else
        error('找不到列: %s', outputCols{i});
    end
end

%% 4. 数据清洗
validIdx = all(isfinite(X), 2) & all(isfinite(Y), 2);
X = X(validIdx, :);
Y = Y(validIdx, :);
fprintf('有效数据: %d 行\n', size(X,1));

%% 5. 去均值（线性化在工作点附近）
X_mean = mean(X);
Y_mean = mean(Y);
dX = X - X_mean;  % 舵面偏差
dY = Y - Y_mean;  % 输出偏差

%% 6. 最小二乘拟合雅可比矩阵
% dY = dX * J'  =>  J' = (dX' * dX) \ (dX' * dY)
% J(3x4): 每行是一个输出对4个输入的偏导数

lambda = 1e-6;  % 正则化
JT = (dX' * dX + lambda * eye(4)) \ (dX' * dY);
J = JT';  % 3x4 雅可比矩阵

%% 7. 量级调整 - 根据您代码的单位
% 您的代码中：
%   - roll_d, pitch_d, yaw_d 单位是度(deg)或弧度(rad)
%   - 舵面偏转量通常是PWM值(1000-2000)或角度(-30~30度)

% 检测舵面输入范围
rudder_range = max(X) - min(X);
output_range = max(Y) - min(Y);

fprintf('\n--- 数据范围 ---\n');
fprintf('舵面范围: [%.1f, %.1f, %.1f, %.1f]\n', rudder_range);
fprintf('输出范围: [%.2f, %.2f, %.2f] (roll, pitch, yaw)\n', output_range);

% 根据您代码的比例关系调整
% 从您的代码看: temp = asinf(...) 输出弧度，后续可能转度
% 如果CSV中角度是度，雅可比单位是 deg/舵面单位

%% 8. 显示结果
fprintf('\n===== 雅可比矩阵 J (3x4) =====\n');
fprintf('        舵1      舵2      舵3      舵4\n');
labels = {'Roll ', 'Pitch', 'Yaw  '};
for i = 1:3
    fprintf('%s: %+8.4f %+8.4f %+8.4f %+8.4f\n', labels{i}, J(i,:));
end

%% 9. 计算拟合质量
Y_pred = dX * J' + Y_mean;
Y_actual = Y(validIdx(1:size(Y,1)),:);  % 原始Y

% R² 值
for i = 1:3
    SS_res = sum((Y(:,i) - (dX * J(i,:)' + Y_mean(i))).^2);
    SS_tot = sum((Y(:,i) - Y_mean(i)).^2);
    R2 = 1 - SS_res / SS_tot;
    fprintf('%s R² = %.4f\n', labels{i}, R2);
end

%% 10. 生成C代码格式输出
fprintf('\n===== C代码格式 =====\n');
fprintf('// 雅可比矩阵: d[roll,pitch,yaw] / d[rudder1,rudder2,rudder3,rudder4]\n');
fprintf('float J[3][4] = {\n');
for i = 1:3
    fprintf('    {%.6ff, %.6ff, %.6ff, %.6ff}', J(i,1), J(i,2), J(i,3), J(i,4));
    if i < 3
        fprintf(',  // %s\n', labels{i});
    else
        fprintf('   // %s\n', labels{i});
    end
end
fprintf('};\n');

%% 11. 工作点信息
fprintf('\n===== 工作点（均值） =====\n');
fprintf('舵面: [%.2f, %.2f, %.2f, %.2f]\n', X_mean);
fprintf('姿态: [%.2f, %.2f, %.2f] (roll, pitch, yaw)\n', Y_mean);

%% 12. 可视化
figure('Name', '雅可比矩阵热力图', 'Position', [100 100 600 400]);
imagesc(J);
colorbar;
colormap(jet);
set(gca, 'XTick', 1:4, 'XTickLabel', {'舵1','舵2','舵3','舵4'});
set(gca, 'YTick', 1:3, 'YTickLabel', {'Roll','Pitch','Yaw'});
title('雅可比矩阵 J = ∂(姿态)/∂(舵面)');
xlabel('舵面输入');
ylabel('姿态输出');

% 在格子中显示数值
for i = 1:3
    for j = 1:4
        text(j, i, sprintf('%.4f', J(i,j)), ...
            'HorizontalAlignment', 'center', ...
            'Color', 'white', 'FontWeight', 'bold');
    end
end

fprintf('\n===== 完成 =====\n');