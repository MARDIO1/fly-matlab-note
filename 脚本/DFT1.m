% 示例信号参数
Fs = 500;            % 采样频率 (Hz)
ts = 0.002;            % 采样时间间隔 (s)
t = 0:ts:1-ts;        % 时间向量 (从 0 到 1 秒)

% input信号
temp=new4800;
table1 = table(temp.Channel1, 'VariableNames', {'Column1'});
table2 = table(temp.Channel2, 'VariableNames', {'Column2'});
% table3 = table(temp.Channel3, 'VariableNames', {'Column2'});
x=table2array(table1);

% 计算信号的长度
N = length(x); 
% 应用 FFT 函数
X = fft(x);
% 计算频率向量
f = (0:N-1)/(N*ts);   % 这将生成从 0 到 (N-1)/ts 的频率向量
% 提取幅度和相位
X_m = abs(X);         % 幅度向量
X_phi = angle(X);     % 相位向量
% 只绘制一半的频谱（因为 FFT 结果是对称的）
X_m_half = X_m(1:N/2+1);
X_phi_half = X_phi(1:N/2+1);
f_half = f(1:N/2+1);
% 绘制幅度谱图
figure;
subplot(2,1,1);
plot(f_half, X_m_half);
title('幅度谱图');
xlabel('频率 (Hz)');
ylabel('幅度');
% 绘制相位谱图
subplot(2,1,2);
plot(f_half, X_phi_half);
title('相位谱图');
xlabel('频率 (Hz)');
ylabel('相位 (rad)');

% x=table2array(table3);
% % 计算信号的长度
% N = length(x); 
% % 应用 FFT 函数
% X = fft(x);
% % 计算频率向量
% f = (0:N-1)/(N*ts);   % 这将生成从 0 到 (N-1)/ts 的频率向量
% % 提取幅度和相位
% X_m = abs(X);         % 幅度向量
% X_phi = angle(X);     % 相位向量
% % 只绘制一半的频谱（因为 FFT 结果是对称的）
% X_m_half = X_m(1:N/2+1);
% X_phi_half = X_phi(1:N/2+1);
% f_half = f(1:N/2+1);
% % 绘制幅度谱图
% figure;
% subplot(2,1,1);
% plot(f_half, X_m_half);
% title('幅度谱图');
% xlabel('频率 (Hz)');
% ylabel('幅度');
% % 绘制相位谱图
% subplot(2,1,2);
% plot(f_half, X_phi_half);
% title('相位谱图');
% xlabel('频率 (Hz)');
% ylabel('相位 (rad)');
% 
% x=table2array(table2);
% % 计算信号的长度
% N = length(x); 
% % 应用 FFT 函数
% X = fft(x);
% % 计算频率向量
% f = (0:N-1)/(N*ts);   % 这将生成从 0 到 (N-1)/ts 的频率向量
% % 提取幅度和相位
% X_m = abs(X);         % 幅度向量
% X_phi = angle(X);     % 相位向量
% % 只绘制一半的频谱（因为 FFT 结果是对称的）
% X_m_half = X_m(1:N/2+1);
% X_phi_half = X_phi(1:N/2+1);
% f_half = f(1:N/2+1);
% % 绘制幅度谱图
% figure;
% subplot(2,1,1);
% plot(f_half, X_m_half);
% title('幅度谱图');
% xlabel('频率 (Hz)');
% ylabel('幅度');
% % 绘制相位谱图
% subplot(2,1,2);
% plot(f_half, X_phi_half);
% title('相位谱图');
% xlabel('频率 (Hz)');
% ylabel('相位 (rad)');
% %%
% function filter = initBandStopFilter(fs, notchFreq, Q)
%     % 初始化滤波器结构体
%     filter = Biquad();
% 
%     % 设计频率预畸变
%     omega_d = 2 * pi * notchFreq / fs;   % 数字频率
%     omega_a = 2 * tan(omega_d / 2);      % 预畸变后的模拟频率
% 
%     % 计算中间变量
%     alpha = sin(omega_a) / (2 * Q);
%     cosw = cos(omega_a);
% 
%     % 计算未归一化系数
%     b0 = 1.0;
%     b1 = -2 * cosw;
%     b2 = 1.0;
%     a0 = 1 + alpha;
%     a1 = -2 * cosw;
%     a2 = 1 - alpha;
% 
%     % 归一化系数
%     filter.b0 = b0 / a0;
%     filter.b1 = b1 / a0;
%     filter.b2 = b2 / a0;
%     filter.a1 = a1 / a0;
%     filter.a2 = a2 / a0;
% end
% 
% function [filter, output] = processSample(filter, input)
%     % 处理单个样本
%     output = filter.b0 * input + filter.b1 * filter.x1 + filter.b2 * filter.x2 ...
%            - filter.a1 * filter.y1 - filter.a2 * filter.y2;
% 
%     % 更新状态
%     filter.x2 = filter.x1;
%     filter.x1 = input;
%     filter.y2 = filter.y1;
%     filter.y1 = output;
% end
% %%
% %% 参数设置
% fs = 500;         % 采样率500Hz
% notchFreq = 80;   % 带阻中心频率80Hz
% Q = 30;             % 品质因数
% duration = 1;     % 信号时长1秒
% 
% %% 生成测试信号
% t = 0:1/fs:duration-1/fs;
% f1 = 80;          % 需要滤除的频率
% f2 = 10;          % 需要保留的频率
% sig = sin(2*pi*f1*t) + 0.5*sin(2*pi*f2*t);
% 
% %% 初始化滤波器
% bqf = initBandStopFilter(fs, notchFreq, Q);
% %% 处理信号
% output = zeros(size(sig));
% for i = 1:length(sig)
%     [bqf, output(i)] = processSample(bqf, sig(i));
% end
% 
% %% 可视化结果
% % 时域波形
% figure
% subplot(2,1,1)
% plot(t, sig)
% title('原始信号')
% xlabel('时间 (s)')
% subplot(2,1,2)
% plot(t, output)
% title('滤波后信号')
% xlabel('时间 (s)')
