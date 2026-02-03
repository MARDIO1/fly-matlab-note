% 1. 信号对齐处理
delay_r = 4;
delay_p = 2;

% 长度对齐：以最长延迟为准
max_d = max(delay_r, delay_p);
N = length(ax);

% 提取对齐后的加速度
ax_a = ax(delay_r + 1 : end); 
ay_a = ay(delay_p + 1 : end);
az_a = az(delay_r + 1 : end); % Yaw通常随Roll延迟

% 提取对应的输入 (需保证长度一致)
% U 矩阵取从 1 到 N-max_d
valid_len = length(ax_a) - (max_d - delay_r); 
% 统一截取以对齐
idx_end = 1000; % 假设取前1000点，或用 min 长度
L = min([length(ax_a), length(ay_a), length(az_a), (N-max_d)]);

U_mat = [data_subset.surface_FR(1:L), data_subset.surface_FL(1:L), ...
         data_subset.surface_BL(1:L), data_subset.surface_BR(1:L)];

%% 最终版：强行对称性约束辨识
% 假设已经有了对齐后的 ax_a, ay_a, az_a 和 U_mat

% 1. 构造对称特征向量
% 前翼差动 (Roll)
Front_Diff = U_mat(:,1) - U_mat(:,2); 
% 尾翼差动 (Roll/Yaw)
Back_Diff  = U_mat(:,3) - U_mat(:,4); 
% 前翼同向 (Pitch)
Front_Sum  = U_mat(:,1) + U_mat(:,2);
% 尾翼同向 (Pitch)
Back_Sum   = U_mat(:,3) + U_mat(:,4);

% 2. 重新辨识各轴增益
% Roll: alpha_x = K_fr * Front_Diff + K_bk * Back_Diff
U_roll = [Front_Diff, Back_Diff];
K_roll = U_roll \ ax_a(1:L);

% Pitch: alpha_y = K_fp * Front_Sum + K_bp * Back_Sum
U_pitch = [Front_Sum, Back_Sum];
K_pitch = U_pitch \ ay_a(1:L);

% Yaw: alpha_z = K_fy * Front_Diff + K_by * Back_Diff
U_yaw = [Front_Diff, Back_Diff];
K_yaw = U_yaw \ az_a(1:L);

% 3. 强制构造完美的物理对称矩阵
% 逻辑：K_roll(1) 是前翼总滚转能力，平分给 FR 和 -FL
JM_Final = zeros(3,4);

% Roll 轴
JM_Final(1,1) =  K_roll(1);  % FR
JM_Final(1,2) = -K_roll(1);  % FL
JM_Final(1,3) =  K_roll(2);  % BL
JM_Final(1,4) = -K_roll(2);  % BR

% Pitch 轴
JM_Final(2,1) = K_pitch(1); % FR
JM_Final(2,2) = K_pitch(1); % FL
JM_Final(2,3) = K_pitch(2); % BL
JM_Final(2,4) = K_pitch(2); % BR

% Yaw 轴
JM_Final(3,1) =  K_yaw(1);  % FR
JM_Final(3,2) = -K_yaw(1);  % FL
JM_Final(3,3) =  K_yaw(2);  % BL
JM_Final(3,4) = -K_yaw(2);  % BR

disp('--- 强制物理对称后的雅可比矩阵 ---');
fprintf('       FR          FL          BL          BR\n');
disp(JM_Final);