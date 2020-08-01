
function EKFSLAM()
%% 初始化工作

close all;
clearvars;
 
time = 0;
endtime = 20;
global dt;
dt = 0.1;
nSteps = ceil((endtime - time)/dt);% 步数
 
% 储存计算结果
result.time = [];
result.xTrue = [];
result.xd = [];
result.xEst = [];
result.z = [];
result.PEst = [];
result.u = [];

% 初始化位姿
xEst = [5 5 toRadian(45)]'; % 估计值
global PoseSize;PoseSize = 3;% 位姿维数，3
global LMSize;LMSize = 2; % 地标维数，2
xTrue = xEst; % 真值 
xd = xTrue; % 里程计值
 
R = diag([0.1 0.1 toRadian(1)]).^2; % 预测z的噪声协方差矩阵                   changed, [0.2 0.2 toRadian(1)]
global Q; Q = diag([1 toRadian(5)]).^2; % 观测x的噪声协方差矩阵，[米，弧度]

% 模拟的参数
global Qsigma; Qsigma = diag([0.1 toRadian(20)]).^2;
global Rsigma; Rsigma = diag([0.1 toRadian(1)]).^2;

% 地标构建
LM = [];
MapCloud = CreateMap_SmallVersion(); % 同样的地图，点更少
for i = 1:length(MapCloud)
    LM = [LM; MapCloud(i).x, MapCloud(i).y];
end
% LM=[0 15; 10 0; 15 20];
  
MAX_RANGE = 10;% 默认可以观测到最远的物体

alpha = 1;% 地标识别用距离阈值

PEst = eye(3); 
initP = eye(2) * 1000;
 
tic;
%% 主循环函数

for i = 1 : nSteps
    
    i % 显示当前在第几步，后期可删掉
    time = time + dt;
    u = [1; 0]; % 控制量[v; w]
    [z, xTrue, xd, u] = Observation(xTrue, xd, u, LM, MAX_RANGE); % 观测

    % EKF SLAM: 预测
    xEst = f(xEst, u);
    [G, Fx] = jacobF(xEst, u);
    PEst = G' * PEst * G + Fx' * R * Fx;
    
    % EKF SLAM: 更新
    for iz = 1 : length(z(:, 1))% 每个观测值
        % 把观测值作为地标追加
        zl = CalcLMPosiFromZ(xEst, z(iz, :));% 根据观测值计算地标位置
        % 增加状态向量和协方差阵
        xAug = [xEst; zl];
        PAug = [PEst zeros(length(xEst), LMSize);
              zeros(LMSize, length(xEst)) initP];
        
        mdist = []; % 距离储存
        for il = 1 : GetnLM(xAug) % 对各个地标
            if il == GetnLM(xAug)
                mdist = [mdist alpha]; % 新添加的地标的距离，初始化参数值
            else
                lm = xAug(4 + 2 * (il-1) : 5 + 2 * (il-1));
                [y, S, H] = CalcInnovation(lm, xAug, PAug, z(iz, 1:2), il);
                mdist = [mdist y'*inv(S)*y]; % 计算距离
            end
        end
        
        [~, I] = min(mdist); % 与距离最近的相关联 [C, I]
        if I == GetnLM(xAug) % 若追加的观测距离最小，则将其作为（新）地标
            %disp('New LM')
            xEst = xAug;
            PEst = PAug;
        end
        
        lm = xEst(4 + 2 * (I - 1) : 5 + 2 * (I - 1));% 获得相关联的地标数据
        % innov计算
        [y, S, H] = CalcInnovation(lm, xEst, PEst, z(iz, 1:2), I);
        K = PEst * H' * inv(S);
        xEst = xEst + K * y;
        PEst = (eye(size(xEst, 1)) - K * H) * PEst;
    end
    xEst(3) = PI2PI(xEst(3));% 调整角度至-pi到pi
    
    % 仿真结果
    result.time = [result.time; time];
    result.xTrue = [result.xTrue; xTrue'];
    result.xd = [result.xd; xd'];
    result.xEst = [result.xEst; xEst(1:3)'];
    result.u = [result.u; u'];
    
    % 绘制
    if rem(i,10) == 0 
        Animation(result, xTrue, LM, z, xEst, zl);
    end
end
toc
save('xEst', 'xEst');
DrawGraph(result, xEst, LM);
%% 辅助函数

function [y, S, H] = CalcInnovation(lm, xEst, PEst, z, LMId)
global Q;
delta = lm - xEst(1:2);
q=  delta' * delta;
zangle = atan2(delta(2), delta(1)) - xEst(3);
zp = [sqrt(q) PI2PI(zangle)];% 观测值预测
y = (z - zp)';
H = jacobH(q, delta, xEst, LMId);
S = H * PEst * H' + Q;

function n = GetnLM(xEst)
% 计算当前地标数
n = (length(xEst) - 3) / 2;

function zl = CalcLMPosiFromZ(x, z)
% 根据观测值计算LM位置
zl = x(1:2) + [z(1) * cos(x(3) + z(2)); z(1)*  sin(x(3) + z(2))];

function Animation(result, xTrue, LM, z, xEst, zl)
hold off;
plot(result.xTrue(:,1), result.xTrue(:,2), '.b'); hold on;
plot(LM(:,1), LM(:,2), 'bo', 'MarkerSize', 10); hold on;
% 观测线表示
if~isempty(z)
    for iz = 1:length(z(:,1))
        ray = [xTrue(1:2)'; z(iz,3:4)];
        plot(ray(:,1), ray(:,2), '-r'); hold on;
    end
end
% SLAM地图表示
for il = 1:GetnLM(xEst)
    plot(xEst(4 + 2 * (il - 1)), xEst(5 + 2 * (il - 1)), '.c'); hold on;
end
plot(zl(1,:), zl(2,:), '.b'); hold on;
plot(result.xd(:,1), result.xd(:,2), '.k'); hold on;
plot(result.xEst(:,1), result.xEst(:,2), '.r'); hold on;
arrow = 0.5;
x = result.xEst(end, :);
quiver(x(1), x(2), arrow * cos(x(3)), arrow * sin(x(3)), 'ok'); hold on;
axis equal;
grid on;

drawnow;

function x = f(x, u)
% 运动学模型
global dt;
global PoseSize;
global LMSize;
 
F = horzcat(eye(PoseSize), zeros(PoseSize, LMSize * GetnLM(x)));
B = [dt*cos(x(3)) 0; dt*sin(x(3)) 0; 0 dt]; 
x = x + F' * B * u; % x = A*x + Bu
x(3) = PI2PI(x(3));% 调整角度

function [G,Fx] = jacobF(x, u)
% 计算运动模型的雅克比矩阵
global dt;
global PoseSize;
global LMSize;

Fx = horzcat(eye(PoseSize), zeros(PoseSize, LMSize*GetnLM(x)));
 
jF=[0 0 -dt*u(1)*sin(x(3))
    0 0 dt*u(1)*cos(x(3))
    0 0 0];

G=eye(length(x)) + Fx' * jF * Fx;

function H = jacobH(q, delta, x, i)
% 计算观测模型的雅克比矩阵
sq = sqrt(q);
G = [-sq*delta(1) -sq*delta(2) 0 sq*delta(1) sq*delta(2);
    delta(2)    -delta(1)   -1 -delta(2)    delta(1)];
G = G / q;
F = [eye(3) zeros(3,2*GetnLM(x));
   zeros(2,3) zeros(2,2*(i-1)) eye(2) zeros(2,2*GetnLM(x)-2*i)];
H = G * F;

function [z, x, xd, u] = Observation(x, xd, u, LM ,MAX_RANGE)
global Qsigma;
global Rsigma;
 
x = f(x, u);% 真值
u = u + Qsigma * randn(2, 1);% 添加噪声
xd = f(xd, u);% 纯里程计
% 仿真观测
z = [];
for iz = 1:length(LM(:,1))
    % 地标位置转化到机器人坐标系
    yaw = zeros(3,1);
    yaw(3) = -x(3);
    localLM = HomogeneousTransformation2D(LM(iz,:) - x(1:2)', yaw');
    d = norm(localLM); % 距离
    if d < MAX_RANGE
        noise = Rsigma * randn(2,1);
        z = [z; [d+noise(1) PI2PI(atan2(localLM(2),localLM(1))+noise(2)) LM(iz,:)]];
    end
end

function DrawGraph(result, xEst, LM)
figure(1);
hold off;
x=[result.xTrue(:,1:2) result.xEst(:,1:2)];
set(gca, 'fontsize', 16, 'fontname', 'times');
plot(x(:,1), x(:,2),'-b','linewidth', 4); hold on;
plot(result.xd(:,1), result.xd(:,2),'-k','linewidth', 4); hold on;
plot(x(:,3), x(:,4),'-r','linewidth', 4); hold on;
plot(LM(:,1),LM(:,2),'bo','MarkerSize',10); hold on;% 实际地标位置
% 预测地标位置
for il = 1 : GetnLM(xEst);
    plot(xEst(4 + 2 * (il - 1)), xEst(5 + 2 * (il - 1)), 'ro'); hold on;
end
title('EKF SLAM Result', 'fontsize', 16, 'fontname', 'times');
xlabel('X (m)', 'fontsize', 16, 'fontname', 'times');
ylabel('Y (m)', 'fontsize', 16, 'fontname', 'times');
legend('真正路线','里程计路线','EKF SLAM路线','实际地标','预测地标');
grid on;
axis equal;

function angle = PI2PI(angle)
% 将角度调整至 -pi到pi
angle = mod(angle, 2*pi);
i = find(angle > pi);
angle(i) = angle(i) - 2*pi;
i = find(angle < -pi);
angle(i) = angle(i) + 2*pi;

function out = HomogeneousTransformation2D(in, base, mode)
Rot=[cos(base(3)) sin(base(3)); -sin(base(3)) cos(base(3))]; % 旋转矩阵
Nin = size(in);
baseMat = repmat(base(1:2),Nin(1),1);

if Nin(2) >= 3
    inxy = in(:,1:2);
    inOther = in(:,3:end);
    in = inxy;
end
if nargin == 2 || mode == 0 % 旋转→平移。else部分反过来
    out = baseMat + in * Rot;
else
    out = (baseMat + in) * Rot;
end
if Nin(2) >= 3
    out = [out inOther];
end

function radian = toRadian(degree)
radian = degree / 180 * pi;

function degree = toDegree(radian)
degree = radian / pi * 180;
