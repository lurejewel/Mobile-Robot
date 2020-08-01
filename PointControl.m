% 作者：lurejewel
% 上次编辑：2020/7/25
% 功能：定点跟踪。由基于极坐标变换的非线性控制器，根据误差位姿计算出速度和角速度，再转化为左前轮转速和偏角

function [dot_phi, beta] = PointControl(currPos, goalPos, robot_model)

% 输入：
% currPos：当前位姿（惯性XY坐标系）
% goalPos：目标位姿（惯性XY坐标系）
% robot_model：机器人模型参数
% 输出：
% dot_phi：左前轮转速
% beta：左前轮偏角

% 机器人参数
b = robot_model.axial_length;
a = robot_model.wheelspan;
r = robot_model.radius;

% 系统开环误差
x_err = currPos.x - goalPos.x;
y_err = currPos.y - goalPos.y;
theta_err = currPos.theta - goalPos.theta;

% 转换到极坐标
rho = sqrt(power(x_err,2) + power(y_err,2));
alpha = atan2(-y_err, -x_err) - currPos.theta;% 之前正弦和点控是减theta_err
if alpha > pi
    alpha = alpha - 2*pi;
elseif alpha < -pi
    alpha = alpha + 2*pi;
end
beta_ = -alpha - theta_err;

% 非线性控制器设计
k_rho = 1;
k_beta = 1;
k_alpha = 1;

% 机器人线速度和角速度
v = k_rho * rho * cos(alpha);
% w = k_alpha * rho * alpha + k_beta * beta_;
% v = - k_rho * (x_err * cos(atan2(-y_err, -x_err)) + y_err * sin(atan2(-y_err, -x_err)));
if alpha ~= 0
    w = k_alpha * alpha + k_rho * sin(alpha) * cos(alpha) / alpha * (alpha - k_beta * beta_);
else
    w = k_rho * k_beta * beta_;
end
% 分段限速，提高最终的精度
v_lim1 = 0.5;
v_lim2 = 0.1;
rho_lim = 1;
if rho >= rho_lim
    v_lim = v_lim1;
else
    v_lim = v_lim2;
end
if v < -v_lim
    v = -v_lim;
elseif v > v_lim
    v = v_lim;
end
if w < -1
    w = -1;
elseif w > 1
    w = 1;
end

% 偏角和左前轮转速
if w ~= 0
    beta = abs(atan(b / (abs(v/w)-a/2))) * w / abs(w);
    dot_phi = abs(w * b / (abs(sin(beta)) * r)) * v / abs(v);
else
    beta = 0;
    dot_phi = v / r;
end

end
