%% =====================================================================
%  X型翼飞行器 - 物理约束雅可比矩阵拟合
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
fprintf('===== X型翼雅可比矩阵拟合 =====\n');
fprintf('已选择 %d 个文件\n\n', nFiles);

%% 2. 列名配置 (根据您的C代码)
% 输入: 舵面角度 [FR, FL, BL, BR]
inputCols = {'surface_d_FR', 'surface_d_FL', 'surface_d_BL', 'surface_d_BR'};
% 备选列名格式
inputCols_alt = {'surface_d[0]', 'surface_d[1]', 'surface_d[2]', 'surface_d[3]'};

% 输出: 机体角度/角速度 [Roll, Pitch, Yaw]
outputCols_angle = {'angle_d_Roll', 'angle_d_Pitch', 'angle_d_Yaw'};
outputCols_gyro = {'gyro_radps_Roll', 'gyro_radps_Pitch', 'gyro_radps_Yaw'};
outputCols_alt = {'angle_d[0]', 'angle_d[1]', 'angle_d[2]'};

%% 3. 合并所有文件数据
X_all = [];  % 舵面输入 [FR, FL, BL, BR]
Y_angle = [];  % 角度输出
Y_gyro = [];   % 角速度输出

for f = 1:nFiles
    filepath = fullfile(path, files{f});
    data = readtable(filepath, 'VariableNamingRule', 'preserve');
    
    fprintf('文件%d: %s (%d行)\n', f, files{f}, height(data));
    
    % 显示列名帮助调试
    if f == 1
        fprintf('  检测到的列名: ');
        disp(data.Properties.VariableNames);
    end
    
    % 尝试提取输入数据
    X = zeros(height(data), 4);
    colNames = data.Properties.VariableNames;
    
    % 智能匹配列名
    for i = 1:4
        matched = false;
        % 尝试多种可能的列名
        possibleNames = {inputCols{i}, inputCols_alt{i}, ...
            sprintf('surface_d_%d', i-1), sprintf('surface_d(%d)', i-1)};
        
        for pn = possibleNames
            idx = find(contains(colNames, pn{1}, 'IgnoreCase', true), 1);
            if ~isempty(idx)
                X(:, i) = data{:, idx};
                matched = true;
                break;
            end
        end
        
        if ~matched
            % 按位置提取（假设surface_d连续存储）
            surfaceIdx = find(contains(colNames, 'surface', 'IgnoreCase', true));
            if length(surfaceIdx) >= 4
                X(:, i) = data{:, surfaceIdx(i)};
            end
        end
    end
    
    % 提取输出数据 - 角度
    Y_ang = zeros(height(data), 3);
    Y_gyr = zeros(height(data), 3);
    
    angleIdx = find(contains(colNames, 'angle_d', 'IgnoreCase', true));
    gyroIdx = find(contains(colNames, 'gyro', 'IgnoreCase', true));
    
    if length(angleIdx) >= 3
        for i = 1:3
            Y_ang(:, i) = data{:, angleIdx(i)};
        end
    end
    
    if length(gyroIdx) >= 3
        for i = 1:3
            Y_gyr(:, i) = data{:, gyroIdx(i)};
        end
    end
    
    % 清洗数据
    validIdx = all(isfinite(X), 2) & all(isfinite(Y_ang), 2);
    
    X_all = [X_all; X(validIdx, :)];
    Y_angle = [Y_angle; Y_ang(validIdx, :)];
    Y_gyro = [Y_gyro; Y_gyr(validIdx, :)];
end

fprintf('\n总数据点: %d\n', size(X_all, 1));

%% 4. 数据预处理
% 去均值（工作点线性化）
X_mean = mean(X_all);
Y_angle_mean = mean(Y_angle);
Y_gyro_mean = mean(Y_gyro);

dX = X_all - X_mean;
dY_angle = Y_angle - Y_angle_mean;
dY_gyro = Y_gyro - Y_gyro_mean;

fprintf('\n舵面工作点 [FR, FL, BL, BR]: [%.2f, %.2f, %.2f, %.2f] deg\n', X_mean);
fprintf('角度工作点 [R, P, Y]: [%.2f, %.2f, %.2f] deg\n', Y_angle_mean);

%% 5. 检查数据范围和激励
fprintf('\n--- 数据范围检查 ---\n');
fprintf('舵面变化范围:\n');
for i = 1:4
    names = {'FR', 'FL', 'BL', 'BR'};
    fprintf('  %s: [%.2f, %.2f], std=%.2f deg\n', names{i}, ...
        min(dX(:,i)), max(dX(:,i)), std(dX(:,i)));
end

fprintf('角度变化范围:\n');
names = {'Roll', 'Pitch', 'Yaw'};
for i = 1:3
    fprintf('  %s: [%.2f, %.2f], std=%.2f deg\n', names{i}, ...
        min(dY_angle(:,i)), max(dY_angle(:,i)), std(dY_angle(:,i)));
end

%% 6. 方法A: 无约束最小二乘拟合
fprintf('\n===== 方法A: 无约束拟合 =====\n');

lambda = 1e-6;
JT_unconstrained = (dX' * dX + lambda * eye(4)) \ (dX' * dY_angle);
J_unconstrained = JT_unconstrained';

printJacobian(J_unconstrained, '无约束');
analyzeJacobian(J_unconstrained);

%% 7. 方法B: 物理对称约束拟合 (推荐)
% 基于X型翼对称性:
%   Roll:  J(1,FR) ≈ -J(1,FL), J(1,BR) ≈ -J(1,BL)  (左右反对称)
%   Pitch: J(2,FR) ≈ J(2,FL), J(2,BR) ≈ J(2,BL)   (左右对称)
%   Yaw:   J(3,FR) ≈ -J(3,FL) ≈ -J(3,BR) ≈ J(3,BL) (对角反对称)

fprintf('\n===== 方法B: 物理约束拟合 =====\n');

% 构建约束矩阵
% 变量顺序: [a_roll_front, a_roll_back, a_pitch_front, a_pitch_back, a_yaw]
% 
% Roll:  J = [+a1, -a1, -a2, +a2]  (FR正, FL负, BL负, BR正)
% Pitch: J = [+a3, +a3, -a4, -a4]  (前正, 后负)
% Yaw:   J = [+a5, -a5, +a5, -a5]  (对角同号)

% 设计矩阵映射: 5个独立参数 -> 12个雅可比元素
% θ = [roll_front, roll_back, pitch_front, pitch_back, yaw]

% Roll行: [+θ1, -θ1, -θ2, +θ2]
% Pitch行: [+θ3, +θ3, -θ4, -θ4]  
% Yaw行: [+θ5, -θ5, +θ5, -θ5]

% 构建增广设计矩阵
n = size(dX, 1);
A_constrained = zeros(3*n, 5);

for i = 1:n
    x = dX(i, :);  % [FR, FL, BL, BR]
    
    % Roll方程: dy_roll = θ1*(FR-FL) + θ2*(BR-BL)
    A_constrained(i, 1) = x(1) - x(2);        % θ1: FR-FL
    A_constrained(i, 2) = x(4) - x(3);        % θ2: BR-BL
    
    % Pitch方程: dy_pitch = θ3*(FR+FL) + θ4*(-BR-BL)
    A_constrained(n+i, 3) = x(1) + x(2);      % θ3: FR+FL
    A_constrained(n+i, 4) = -(x(3) + x(4));   % θ4: -(BL+BR)
    
    % Yaw方程: dy_yaw = θ5*(FR-FL+BL-BR)
    A_constrained(2*n+i, 5) = x(1) - x(2) + x(3) - x(4);  % θ5: 对角差
end

b_constrained = [dY_angle(:,1); dY_angle(:,2); dY_angle(:,3)];

% 求解约束参数
theta = (A_constrained' * A_constrained + 1e-6*eye(5)) \ (A_constrained' * b_constrained);

fprintf('约束参数:\n');
fprintf('  θ1 (Roll前翼): %.6f\n', theta(1));
fprintf('  θ2 (Roll后翼): %.6f\n', theta(2));
fprintf('  θ3 (Pitch前翼): %.6f\n', theta(3));
fprintf('  θ4 (Pitch后翼): %.6f\n', theta(4));
fprintf('  θ5 (Yaw): %.6f\n', theta(5));

% 重构雅可比矩阵
%        FR          FL          BL          BR
J_constrained = [
    +theta(1),  -theta(1),  -theta(2),  +theta(2);   % Roll
    +theta(3),  +theta(3),  -theta(4),  -theta(4);   % Pitch
    +theta(5),  -theta(5),  +theta(5),  -theta(5)    % Yaw
];

printJacobian(J_constrained, '物理约束');

%% 8. 计算拟合质量
fprintf('\n===== 拟合质量对比 =====\n');

% 无约束R²
Y_pred_unc = dX * J_unconstrained';
for i = 1:3
    SS_res = sum((dY_angle(:,i) - Y_pred_unc(:,i)).^2);
    SS_tot = sum(dY_angle(:,i).^2) + 1e-10;
    R2_unc(i) = 1 - SS_res / SS_tot;
end

% 约束R²
Y_pred_con = dX * J_constrained';
for i = 1:3
    SS_res = sum((dY_angle(:,i) - Y_pred_con(:,i)).^2);
    SS_tot = sum(dY_angle(:,i).^2) + 1e-10;
    R2_con(i) = 1 - SS_res / SS_tot;
end

fprintf('           Roll    Pitch    Yaw\n');
fprintf('无约束 R²: %.4f   %.4f   %.4f\n', R2_unc);
fprintf('约束后 R²: %.4f   %.4f   %.4f\n', R2_con);

%% 9. 方法C: 使用角速度差分估计（更直接）
fprintf('\n===== 方法C: 角速度响应拟合 =====\n');

% 角速度对舵面的响应更直接
if ~isempty(Y_gyro) && any(Y_gyro(:) ~= 0)
    dY_gyro_clean = Y_gyro - mean(Y_gyro);
    
    JT_gyro = (dX' * dX + 1e-6*eye(4)) \ (dX' * dY_gyro_clean);
    J_gyro = JT_gyro';
    
    printJacobian(J_gyro, '角速度响应');
    
    % 如果角速度数据好，可能更可靠
    for i = 1:3
        Y_pred = dX * JT_gyro(:,i);
        SS_res = sum((dY_gyro_clean(:,i) - Y_pred).^2);
        SS_tot = sum(dY_gyro_clean(:,i).^2) + 1e-10;
        R2_gyro(i) = 1 - SS_res / SS_tot;
    end
    fprintf('角速度 R²: %.4f   %.4f   %.4f\n', R2_gyro);
else
    fprintf('角速度数据不可用\n');
    J_gyro = J_constrained;
end

%% 10. 选择最佳结果
fprintf('\n===== 最终推荐 =====\n');

% 综合评估
score_unc = mean(R2_unc);
score_con = mean(R2_con);

if score_con > score_unc * 0.95  % 约束模型R²损失<5%则优先选择
    J_best = J_constrained;
    method_name = '物理约束';
else
    J_best = J_unconstrained;
    method_name = '无约束';
end

fprintf('选择: %s 方法\n', method_name);
printJacobian(J_best, '最终雅可比');

%% 11. 生成C代码
fprintf('\n===== C代码输出 =====\n');

fprintf('// X型翼雅可比矩阵\n');
fprintf('// 舵面顺序: [FR, FL, BL, BR]\n');
fprintf('// 基于 %d 个文件, %d 个数据点\n', nFiles, size(X_all, 1));
fprintf('// 工作点: 舵面=[%.1f, %.1f, %.1f, %.1f] deg\n', X_mean);
fprintf('\n');

fprintf('const float Jacobian[3][4] = {\n');
fprintf('    // FR        FL        BL        BR\n');
labels = {'Roll ', 'Pitch', 'Yaw  '};
for i = 1:3
    fprintf('    {%+.6ff, %+.6ff, %+.6ff, %+.6ff}', J_best(i,:));
    if i < 3, fprintf(','); end
    fprintf('  // %s\n', labels{i});
end
fprintf('};\n');

% 如果用约束方法，也输出简化参数
if strcmp(method_name, '物理约束')
    fprintf('\n// 简化参数（对称模型）\n');
    fprintf('const float k_roll_front = %.6ff;   // FR-FL对roll的影响\n', theta(1));
    fprintf('const float k_roll_back  = %.6ff;   // BR-BL对roll的影响\n', theta(2));
    fprintf('const float k_pitch_front = %.6ff;  // 前翼对pitch的影响\n', theta(3));
    fprintf('const float k_pitch_back  = %.6ff;  // 后翼对pitch的影响\n', theta(4));
    fprintf('const float k_yaw = %.6ff;          // 对角差对yaw的影响\n', theta(5));
end

%% 12. 可视化
figure('Name', 'X型翼雅可比矩阵分析', 'Position', [50 50 1400 800]);

% 雅可比热力图
subplot(2,3,1);
imagesc(J_best);
colorbar;
colormap(bluewhitered(256));  % 蓝-白-红，零点为白色
caxis([-max(abs(J_best(:))), max(abs(J_best(:)))]);
set(gca, 'XTick', 1:4, 'XTickLabel', {'FR','FL','BL','BR'});
set(gca, 'YTick', 1:3, 'YTickLabel', {'Roll','Pitch','Yaw'});
title('雅可比矩阵 (红正蓝负)');
for i = 1:3
    for j = 1:4
        text(j, i, sprintf('%.4f', J_best(i,j)), ...
            'HorizontalAlignment', 'center', 'FontWeight', 'bold');
    end
end

% 物理解释图
subplot(2,3,2);
bar([J_best(1,:); J_best(2,:); J_best(3,:)]');
set(gca, 'XTickLabel', {'FR','FL','BL','BR'});
legend({'Roll', 'Pitch', 'Yaw'}, 'Location', 'best');
ylabel('灵敏度 (deg/deg)');
title('各舵面对各轴的贡献');
grid on;

% 拟合散点图 - Roll
subplot(2,3,3);
Y_pred = dX * J_best';
scatter(dY_angle(:,1), Y_pred(:,1), 5, 'filled', 'MarkerFaceAlpha', 0.3);
hold on;
plot(xlim, xlim, 'r--', 'LineWidth', 2);
xlabel('实际 Roll 变化 (deg)');
ylabel('预测 Roll 变化 (deg)');
title(sprintf('Roll 拟合 (R²=%.3f)', R2_con(1)));
grid on;
axis equal;

% Pitch
subplot(2,3,4);
scatter(dY_angle(:,2), Y_pred(:,2), 5, 'filled', 'MarkerFaceAlpha', 0.3);
hold on;
plot(xlim, xlim, 'r--', 'LineWidth', 2);
xlabel('实际 Pitch 变化 (deg)');
ylabel('预测 Pitch 变化 (deg)');
title(sprintf('Pitch 拟合 (R²=%.3f)', R2_con(2)));
grid on;
axis equal;

% Yaw
subplot(2,3,5);
scatter(dY_angle(:,3), Y_pred(:,3), 5, 'filled', 'MarkerFaceAlpha', 0.3);
hold on;
plot(xlim, xlim, 'r--', 'LineWidth', 2);
xlabel('实际 Yaw 变化 (deg)');
ylabel('预测 Yaw 变化 (deg)');
title(sprintf('Yaw 拟合 (R²=%.3f)', R2_con(3)));
grid on;
axis equal;

% 残差分布
subplot(2,3,6);
residuals = dY_angle - Y_pred;
histogram(residuals(:), 50, 'Normalization', 'pdf');
xlabel('残差 (deg)');
ylabel('概率密度');
title('残差分布');
grid on;

sgtitle(sprintf('X型翼雅可比矩阵分析 (%d文件, %d点, 方法:%s)', ...
    nFiles, size(X_all,1), method_name));

%% 13. 保存结果
result.J_best = J_best;
result.J_unconstrained = J_unconstrained;
result.J_constrained = J_constrained;
result.theta = theta;
result.R2 = R2_con;
result.X_mean = X_mean;
result.Y_mean = Y_angle_mean;
result.nFiles = nFiles;
result.nPoints = size(X_all, 1);

save('xwing_jacobian.mat', 'result');
fprintf('\n结果已保存至 xwing_jacobian.mat\n');

%% =====================================================================
%  辅助函数
%% =====================================================================

function printJacobian(J, name)
    fprintf('\n[%s] 雅可比矩阵:\n', name);
    fprintf('           FR        FL        BL        BR\n');
    labels = {'Roll ', 'Pitch', 'Yaw  '};
    for i = 1:3
        fprintf('%s:  %+9.5f %+9.5f %+9.5f %+9.5f\n', labels{i}, J(i,:));
    end
end

function analyzeJacobian(J)
    fprintf('\n物理合理性检查:\n');
    
    % Roll: FR和BR应该同号（右侧），FL和BL应该同号（左侧），左右反号
    roll_right = J(1,1) + J(1,4);  % FR + BR
    roll_left = J(1,2) + J(1,3);   % FL + BL
    fprintf('  Roll: 右侧=%.4f, 左侧=%.4f (应反号)\n', roll_right, roll_left);
    
    % Pitch: 前翼(FR+FL)和后翼(BL+BR)应该反号
    pitch_front = J(2,1) + J(2,2);
    pitch_back = J(2,3) + J(2,4);
    fprintf('  Pitch: 前翼=%.4f, 后翼=%.4f (应反号)\n', pitch_front, pitch_back);
    
    % Yaw: 对角应该同号 (FR,BL) vs (FL,BR)
    yaw_diag1 = J(3,1) + J(3,3);   % FR + BL
    yaw_diag2 = J(3,2) + J(3,4);   % FL + BR
    fprintf('  Yaw: 对角1(FR+BL)=%.4f, 对角2(FL+BR)=%.4f (应反号)\n', yaw_diag1, yaw_diag2);
end

function c = bluewhitered(n)
    % 生成蓝-白-红颜色图
    if nargin < 1
        n = 256;
    end
    
    half = floor(n/2);
    
    % 蓝到白
    r1 = linspace(0, 1, half)';
    g1 = linspace(0, 1, half)';
    b1 = ones(half, 1);
    
    % 白到红
    r2 = ones(n-half, 1);
    g2 = linspace(1, 0, n-half)';
    b2 = linspace(1, 0, n-half)';
    
    c = [r1 g1 b1; r2 g2 b2];
end