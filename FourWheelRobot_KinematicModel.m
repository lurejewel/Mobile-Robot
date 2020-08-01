% 作者：lurejewel
% 上次编辑: 2020/7/23
% 功能： 输入轮速、轮偏角、机器人参数，根据Ackerman汽车模型输出机器人坐标系下的x,y,θ方向的速度。

function [dot_x, dot_theta] = FourWheelRobot_KinematicModel(dot_phi, beta, robot_model, mode)
% 输入：
% dot_phi: 左前轮转速，范围[0,1]
% beta: 左前轮侧偏角，范围[-1,1]
% robot_model: 机器人参数，包括长length、宽width、轴长axial_length、轮距wheelspan、轮径radius
% 
% 输出：
% dot_x: 机器人坐标系下x方向速度
% dot_theta: 绕z轴转速
% dot_y（机器人坐标系下y方向速度）始终为0，所以不做输出

a = robot_model.wheelspan;
b = robot_model.axial_length;
r = robot_model.radius;

if mode == 'normal'
    
    if beta ~= 0 % 转弯
        if beta < -1
            beta = -1;
        elseif beta > 1
            beta = 1;
        end
        dot_theta = dot_phi * r * sin(beta) / b;
        dot_x = abs(r * sin(beta) / b * (abs(b/tan(beta)) - a/2)) * dot_phi; % 做了改动
    else % 直行
        dot_theta = 0;
        dot_x = dot_phi * r;
    end

elseif mode == 'gaussn'
    
    if beta ~= 0 % 转弯
        if beta < -1
            beta = -1;
        elseif beta > 1
            beta = 1;
        end
        dot_theta = dot_phi * r * sin(beta) / b + normrnd(0.00005, 0.0002);
        dot_x = abs(r * sin(beta) / b * (abs(b/tan(beta)) - a/2) + normrnd(0, 0.05) / dot_phi) * dot_phi;
    else % 直行
        dot_theta = normrnd(0.00005,0.004);
        dot_x = dot_phi * r * (1 + normrnd(0, 0.02));
    end
    
end

end
