function PathTracking_withOdom(traj, robot_model, length_ahead, n)

currPos = traj(1); % 真值路线
odomPos = traj(1); % 里程计路线（乘性高斯噪声）
figure, plot(currPos.x, currPos.y, 'r.');
hold on
plot(odomPos.x, odomPos.y, 'g.');
% DrawRobot(currPos, 'r', robot_model);
for i = 1:n
    plot(traj(i).x, traj(i).y, 'bo');
end

alpha = abs(atan2(traj(2).y - traj(1).y, traj(2).x - traj(1).x) - currPos.theta);
rho = sqrt(power(traj(2).y - traj(1).y, 2) + power(traj(2).x - traj(1).x, 2));

for i = 2:n
    
    while(power(currPos.x-traj(i).x, 2) + power(currPos.y-traj(i).y, 2) + power(currPos.theta-traj(i).theta, 2) >= 0.01 && ~(alpha <= 0.5 && rho <= length_ahead && i ~= n))

        [dot_phi, beta] = PointControl(currPos, traj(i), robot_model); % 误差→控制器→控制量
        if dot_phi > 0.1 % 可以尝试再调大点？
            dot_phi = 0.1;
        elseif dot_phi < -0.1
            dot_phi = -0.1;
        end
        [dot_x, dot_theta] = FourWheelRobot_KinematicModel(dot_phi, beta, robot_model, 'normal'); % 控制量→真值模型→状态量
        [dot_x_odom, dot_theta_odom] = FourWheelRobot_KinematicModel(dot_phi, beta, robot_model, 'gaussn'); % 控制量→里程计模型→状态量
        
%         DrawRobot(currPos, 'w', robot_model); % 擦除上一步的机器人图像
        
        currPos.x = currPos.x + dot_x * cos(currPos.theta); % 更新当前位置（真值）
        currPos.y = currPos.y + dot_x * sin(currPos.theta);
        currPos.theta = currPos.theta + dot_theta;
        if currPos.theta > pi
            currPos.theta = currPos.theta - 2*pi;
        elseif currPos.theta < -pi
            currPos.theta = currPos.theta + 2*pi;
        end
        
        odomPos.x = odomPos.x + dot_x_odom * cos(odomPos.theta); % 更新当前位置（里程计）
        odomPos.y = odomPos.y + dot_x_odom * sin(odomPos.theta);
        odomPos.theta = odomPos.theta + dot_theta_odom;
        if odomPos.theta > pi
            odomPos.theta = odomPos.theta - 2*pi;
        elseif odomPos.theta < -pi
            odomPos.theta = odomPos.theta + 2*pi;
        end
        
%         DrawRobot(currPos, 'r', robot_model); % 绘制当前步的机器人图像
        plot(currPos.x, currPos.y, 'r.');
        plot(odomPos.x, odomPos.y, 'g.');
        
        alpha = abs(atan2(traj(i).y - currPos.y, traj(i).x - currPos.x) - currPos.theta);
        rho = sqrt(power(traj(i).y - currPos.y, 2) + power(traj(i).x - currPos.x, 2));
        
        pause(0.02);
        
    end
    
    alpha = abs(atan2(traj(i+1).y - currPos.y, traj(i).x - currPos.x) - currPos.theta);
    rho = sqrt(power(traj(i+1).y - currPos.y, 2) + power(traj(i+1).x - currPos.x, 2));
    
end

end
