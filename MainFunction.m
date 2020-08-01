% 作者：lurejewel
% 上次编辑：2020/7/23
% 功能：程序主函数。主要实现：

robot_model.length = 2;
robot_model.width = 1;
robot_model.axial_length = 1.8;
robot_model.wheelspan = 0.6;
robot_model.radius = 1;

%% 实验一：根据运动学模型实时走一个轨迹（离散定位点+机器人（方框））

% % 初始位姿
% theta = 0;
% x = 10;
% y = 10;
% 
% % 运动律，以及相应的位姿求解
% for i = 1:100
%     beta(i)= sin(0.1*i);
%     dot_phi(i) = abs(cos(0.1*i) * sin(0.1*i));
%     [dot_x(i), dot_theta(i)] = FourWheelRobot_KinematicModel(dot_phi(i), beta(i), robot_model, 'normal');
% end
% figure, plot(x,y,'r.');
% hold on 
% for i = 1:100
%     Pos.x = x;
%     Pos.y = y;
%     Pos.theta = theta;
%     DrawRobot(Pos, 'w', robot_model);
%     x = x + dot_x(i) * cos(theta);
%     y = y + dot_x(i) * sin(theta);
%     theta = theta + dot_theta(i);
%     Pos.x = x;
%     Pos.y = y;
%     Pos.theta = theta;
%     DrawRobot(Pos, 'r', robot_model);
%     plot(x,y,'r.')
%     pause(0.5);
% end

%% 实验二：机器人定点控制（镇定控制）

% % 初始位姿
% currPos.x = 10;
% currPos.y = 10;
% currPos.theta = 0;
% % 目标位姿
% goalPos.x = 20;
% goalPos.y = 20;
% goalPos.theta = 0;
% 
% % 开始移动
% figure,plot(currPos.x, currPos.y, 'r.');
% hold on
% DrawRobot(currPos, 'r', robot_model);
% DrawRobot(goalPos, 'b', robot_model);
% 
% while(power(currPos.x-goalPos.x, 2) + power(currPos.y-goalPos.y, 2) + power(currPos.theta-goalPos.theta, 2) >= 0.01)
%     
%     [dot_phi, beta] = PointControl(currPos, goalPos, robot_model); % 误差→控制器→控制量
%     [dot_x, dot_theta] = FourWheelRobot_KinematicModel(dot_phi, beta, robot_model, 'normal'); % 控制量→模型→状态量
%     
%     DrawRobot(currPos, 'w', robot_model); % 擦除上一步的机器人图像
% 
%     currPos.x = currPos.x + dot_x * cos(currPos.theta); % 更新当前位置
%     currPos.y = currPos.y + dot_x * sin(currPos.theta);
%     currPos.theta = currPos.theta + dot_theta;
% 
%     DrawRobot(currPos, 'r', robot_model); % 绘制当前步的机器人图像
%     plot(currPos.x, currPos.y, 'r.');
%     
%     pause(0.2);
%     
% end

%% 实验三：机器人路径跟踪(无里程计）

% % 生成正弦型函数路径
% n = 100; % 离散点数
% traj_tmp = []; % 储存离散路径点
% traj = []; % 同上，多了路径的朝向信息
% for i = 1:n
%     path.x = i * 3;
%     path.y = 3 * sin(i/2);
%     traj_tmp = [traj_tmp, path];
% end
% path = traj_tmp(1);
% path.theta = atan2(traj_tmp(2).y - traj_tmp(1).y, traj_tmp(2).x - traj_tmp(1).x);
% traj = [traj, path];
% for i = 2:n-1
%     theta_next = atan2(traj_tmp(i+1).y - traj_tmp(i).y, traj_tmp(i+1).x - traj_tmp(i).x);
%     theta_last = atan2(traj_tmp(i).y - traj_tmp(i-1).y, traj_tmp(i).x - traj_tmp(i-1).x);
%     path = traj_tmp(i); % 前向和后项差分的平均
%     path.theta = (theta_next + theta_last) / 2;
%     traj = [traj, path];
% end
% path = traj_tmp(n);
% path.theta = atan2(traj_tmp(n).y - traj_tmp(n-1).y, traj_tmp(n).x - traj_tmp(n-1).x);
% traj = [traj, path];

% % 生成正方形路径
% n = 100; % 离散点数
% traj_tmp = []; % 储存离散路径点
% traj = [];
% for i = 1:n/4
%     path.x = i * 1.6;
%     path.y = 20;
%     traj_tmp = [traj_tmp, path];
% end
% for i = 1:n/4
%     path.x = 40;
%     path.y = 20 + i * 1.6;
%     traj_tmp = [traj_tmp, path];
% end
% for i = 1:n/4
%     path.x = 40 - i* 1.6;
%     path.y = 60;
%     traj_tmp = [traj_tmp, path];
% end
% for i = 1:n/4
%     path.x = 0;
%     path.y = 60 - i * 1.6;
%     traj_tmp = [traj_tmp, path];
% end
% path = traj_tmp(1);
% path.theta = atan2(traj_tmp(2).y - traj_tmp(1).y, traj_tmp(2).x - traj_tmp(1).x);
% traj = [traj, path];
% for i = 2:n-1
%     theta_next = atan2(traj_tmp(i+1).y - traj_tmp(i).y, traj_tmp(i+1).x - traj_tmp(i).x);
%     theta_last = atan2(traj_tmp(i).y - traj_tmp(i-1).y, traj_tmp(i).x - traj_tmp(i-1).x);
%     path = traj_tmp(i); % 前向和后项差分的平均
%     path.theta = (theta_next + theta_last) / 2;
%     traj = [traj, path];
% end
% path = traj_tmp(n);
% path.theta = atan2(traj_tmp(n).y - traj_tmp(n-1).y, traj_tmp(n).x - traj_tmp(n-1).x);
% traj = [traj, path];
% traj(n/4).theta = traj(n/4-1).theta;
% traj(n/2).theta = traj(n/2-1).theta;
% traj(3*n/4).theta = traj(3*n/4-1).theta;
% 
% % 开始跟踪
% tic
% length_ahead = 1;
% PathTracking(traj, robot_model, length_ahead, n);
% toc
%% 实验四：机器人路径跟踪(带里程计）

% % 生成正弦型函数路径
% n = 100; % 离散点数
% traj_tmp = []; % 储存离散路径点
% traj = []; % 同上，多了路径的朝向信息
% for i = 1:n
%     path.x = i * 3;
%     path.y = 3 * sin(i/2);
%     traj_tmp = [traj_tmp, path];
% end
% path = traj_tmp(1);
% path.theta = atan2(traj_tmp(2).y - traj_tmp(1).y, traj_tmp(2).x - traj_tmp(1).x);
% traj = [traj, path];
% for i = 2:n-1
%     theta_next = atan2(traj_tmp(i+1).y - traj_tmp(i).y, traj_tmp(i+1).x - traj_tmp(i).x);
%     theta_last = atan2(traj_tmp(i).y - traj_tmp(i-1).y, traj_tmp(i).x - traj_tmp(i-1).x);
%     path = traj_tmp(i); % 前向和后项差分的平均
%     path.theta = (theta_next + theta_last) / 2;
%     traj = [traj, path];
% end
% path = traj_tmp(n);
% path.theta = atan2(traj_tmp(n).y - traj_tmp(n-1).y, traj_tmp(n).x - traj_tmp(n-1).x);
% traj = [traj, path];

% % 生成正方形路径
% n = 100; % 离散点数
% traj_tmp = []; % 储存离散路径点
% traj = [];
% for i = 1:n/4
%     path.x = i * 1.6;
%     path.y = 20;
%     traj_tmp = [traj_tmp, path];
% end
% for i = 1:n/4
%     path.x = 40;
%     path.y = 20 + i * 1.6;
%     traj_tmp = [traj_tmp, path];
% end
% for i = 1:n/4
%     path.x = 40 - i* 1.6;
%     path.y = 60;
%     traj_tmp = [traj_tmp, path];
% end
% for i = 1:n/4
%     path.x = 0;
%     path.y = 60 - i * 1.6;
%     traj_tmp = [traj_tmp, path];
% end
% path = traj_tmp(1);
% path.theta = atan2(traj_tmp(2).y - traj_tmp(1).y, traj_tmp(2).x - traj_tmp(1).x);
% traj = [traj, path];
% for i = 2:n-1
%     theta_next = atan2(traj_tmp(i+1).y - traj_tmp(i).y, traj_tmp(i+1).x - traj_tmp(i).x);
%     theta_last = atan2(traj_tmp(i).y - traj_tmp(i-1).y, traj_tmp(i).x - traj_tmp(i-1).x);
%     path = traj_tmp(i); % 前向和后项差分的平均
%     path.theta = (theta_next + theta_last) / 2;
%     traj = [traj, path];
% end
% path = traj_tmp(n);
% path.theta = atan2(traj_tmp(n).y - traj_tmp(n-1).y, traj_tmp(n).x - traj_tmp(n-1).x);
% traj = [traj, path];
% traj(n/4).theta = traj(n/4-1).theta;
% traj(n/2).theta = traj(n/2-1).theta;
% traj(3*n/4).theta = traj(3*n/4-1).theta;
% 
% % 开始跟踪
% length_ahead = 1;
% PathTracking_withOdom(traj, robot_model, length_ahead, n);

%% 实验五：ICP静态定位（激光无噪声）

% MapCloud = CreateMap(); % 生成地图点集
% currPos.x = 10; currPos.y = 10; currPos.theta = 0; % 拟定位点
% LaserCloud = GetPointSet(currPos, MapCloud); % 生成激光点云数据
% 
% a = [];
% for i = 1:length(LaserCloud)
%     a = [a; LaserCloud(i).x, LaserCloud(i).y];
% end
% 
% R = [1,0,0;0,1,0;0,0,1]; % 初始化旋转阵
% t = [0;0;0]; % 初始化平移阵
% Loss = 99999; % 初始化目标函数
% count = 1;
% while(Loss > 30) % 迭代寻找R和t
%     
%     % 将MapCloud中的点进行齐次变换
%     for i = 1 : length(MapCloud)
%         tmp = [MapCloud(i).x; MapCloud(i).y; 0];
%         tmp = R * tmp + t;
%         MapCloud(i).x = tmp(1);
%         MapCloud(i).y = tmp(2);
%     end
%     
%     for i= 1 : length(MapCloud)
%         XX(i) = MapCloud(i).x;
%         YY(i) = MapCloud(i).y;
%     end
%     
%     figure, plot(XX, YY, 'bo'), title(strcat('count=',count));
%     hold on
%     plot(a(:,1), a(:,2), 'ro');
%     
%     % 找到MapCloud中最接近LaserCloud的628个点
%     for i = 1 : 628
%         for j = 1 : length(MapCloud)
%             dist(j) = power(MapCloud(j).x-LaserCloud(i).x, 2) + power(MapCloud(j).y-LaserCloud(i).y, 2);
%         end
%         [~, id] = min(dist); % MapCloud中最接近LaserCloud(i)的点
%         matchPoint(i) = id; % 记录对应关系
%     end
%     
%     % 计算LaserCloud和MapCloud中对应点的中心点
%     for i = 1 : 628
%         tmpQ_x(i) = LaserCloud(i).x;
%         tmpQ_y(i) = LaserCloud(i).y;
%         tmpP_x(i) = MapCloud(matchPoint(i)).x;
%         tmpP_y(i) = MapCloud(matchPoint(i)).y;
%     end
%     pCenter = [mean(tmpP_x); mean(tmpP_y); 0]; % 真值中心点
%     qCenter = [mean(tmpQ_x); mean(tmpQ_y); 0]; % 激光点云中心点
%     
%     % 去中心化
%     X = zeros(3, 628);
%     Y = zeros(3, 628);
%     for i = 1 : 628
%         X(1, i) = tmpP_x(i) - pCenter(1); % 去中心化后的真值点云
%         X(2, i) = tmpP_y(i) - pCenter(2);
%         X(3, i) = 0;
%         Y(1, i) = tmpQ_x(i) - pCenter(1); % 去中心化后的激光点云
%         Y(2, i) = tmpQ_y(i) - pCenter(2);
%         Y(3, i) = 0;
%     end
%     
%     % SVD
%     S = X * Y'; % 这里权重阵W是单位阵
%     [U, Sigma, V] = svd(S);
%     R = V' * U;
%     t = qCenter - R * pCenter;
%     
%     % 计算目标函数Loss
%     Loss = 0;
%     for i = 1:628
%         p = [MapCloud(matchPoint(i)).x; MapCloud(matchPoint(i)).y; 0];
%         q = [LaserCloud(i).x; LaserCloud(i).y; 0];
%         tmp = R * p + t - q;
%         Loss = Loss + power(tmp(1), 2) + power(tmp(2), 2);
%     end
%     count = count + 1
% end
