function LaserCloud = GetPointSet(Pos, MapCloud)

LaserCloud = [];
T = [cos(Pos.theta), sin(Pos.theta), 0, -Pos.x; -sin(Pos.theta), cos(Pos.theta), 0, -Pos.y; 0, 0, 1, 0; 0, 0, 0, 1]; % 齐次变换矩阵
% figure, hold on

for i = 1 : 628
   
    theta = 0.01 * i;
    if theta >= pi
        theta = theta - 2 * pi;
    elseif theta <= -pi
        theta = theta + 2*pi;
    end
    crossPoint = crossMap(Pos, theta, MapCloud); % 记录地图中与激光束相交的点
    
    tmp = T * [crossPoint.x; crossPoint.y; 0; 1];
    Point.x = tmp(1);
    Point.y = tmp(2);
    LaserCloud = [LaserCloud, Point];
%     plot(Point.x, Point.y, 'ro');
    
end

end

function crossPoint = crossMap(Pos, theta, MapCloud)

    % 初始化
    crossPoint.x = Pos.x;
    crossPoint.y = Pos.y;
    crossPoint.rho = 999; 
    
    theta_w = Pos.theta + theta; % 世界坐标系下的弧度
    if theta_w >= pi
        theta_w = theta_w - 2*pi;
    elseif theta_w <= -pi
        theta_w = theta_w + 2*pi;
    end
    
    k = tan(theta_w);
    fy = @(x) k*x -k*Pos.x + Pos.y;
    % 长方形
    fy1 = @(x) 15-x; % [3,5]
    fy2 = @(x) 25-x; % [8,10]
    fy3 = @(x) x+5; % [5,10]
    fy4 = @(x) x+9; % [3,8]
    % 三角形
    fy5 = @(x) 5; % [10,16]
    fy6 = @(x) 2*x-15; % [10,12]
    fy7 = @(x) 21-x; % [12,16]
    
    fun = @(x) fy(x) - fy1(x);
    x0 = fsolve(fun, 0);
    if x0 >= 3 && x0 <= 5 % 激光所在直线有交点
        y0 = fy(x0);
        theta_ = atan2(y0-Pos.y, x0-Pos.x);
        if theta_ >= pi
            theta_ = theta_ - 2*pi;
        elseif theta_ <= -pi
            theta_ = theta_ + 2*pi;
        end
        if abs(theta_w - theta_) < 0.01 % 激光所在射线有交点
            rho = power(x0-Pos.x, 2) + power(y0-Pos.y, 2);
            if rho < crossPoint.rho
                crossPoint.x = x0;
                crossPoint.y = y0;
                crossPoint.rho = rho;
            end            
        end
    end
        
    fun = @(x) fy(x) - fy2(x);
    x0 = fsolve(fun, 0);
    if x0 >= 8 && x0 <= 10 % 激光所在直线有交点
        y0 = fy(x0);
        theta_ = atan2(y0-Pos.y, x0-Pos.x);
        if theta_ >= pi
            theta_ = theta_ - 2*pi;
        elseif theta_ <= -pi
            theta_ = theta_ + 2*pi;
        end
        if abs(theta_w - theta_) < 0.01 % 激光所在射线有交点
            rho = power(x0-Pos.x, 2) + power(y0-Pos.y, 2);
            if rho < crossPoint.rho
                crossPoint.x = x0;
                crossPoint.y = y0;
                crossPoint.rho = rho;
            end
        end
    end
    
    fun = @(x) fy(x) - fy3(x);
    x0 = fsolve(fun, 0);
    if x0 >= 5 && x0 <= 10 % 激光所在直线有交点
        y0 = fy(x0);
        theta_ = atan2(y0-Pos.y, x0-Pos.x);
        if theta_ >= pi
            theta_ = theta_ - 2*pi;
        elseif theta_ <= -pi
            theta_ = theta_ + 2*pi;
        end
        if abs(theta_w - theta_) < 0.01 % 激光所在射线有交点
            rho = power(x0-Pos.x, 2) + power(y0-Pos.y, 2);
            if rho < crossPoint.rho
                crossPoint.x = x0;
                crossPoint.y = y0;
                crossPoint.rho = rho;
            end
        end
    end
    
    fun = @(x) fy(x) - fy4(x);
    x0 = fsolve(fun, 0);
    if x0 >= 3 && x0 <= 8 % 激光所在直线有交点
        y0 = fy(x0);
        theta_ = atan2(y0-Pos.y, x0-Pos.x);
        if theta_ >= pi
            theta_ = theta_ - 2*pi;
        elseif theta_ <= -pi
            theta_ = theta_ + 2*pi;
        end
        if abs(theta_w - theta_) < 0.01 % 激光所在射线有交点
            rho = power(x0-Pos.x, 2) + power(y0-Pos.y, 2);
            if rho < crossPoint.rho
                crossPoint.x = x0;
                crossPoint.y = y0;
                crossPoint.rho = rho;
            end
        end
    end
    
    fun = @(x) fy(x) - fy5(x);
    x0 = fsolve(fun, 0);
    if x0 >= 10 && x0 <= 16 % 激光所在直线有交点
        y0 = fy(x0);
        theta_ = atan2(y0-Pos.y, x0-Pos.x);
        if theta_ >= pi
            theta_ = theta_ - 2*pi;
        elseif theta_ <= -pi
            theta_ = theta_ + 2*pi;
        end
        if abs(theta_w - theta_) < 0.01 % 激光所在射线有交点
            rho = power(x0-Pos.x, 2) + power(y0-Pos.y, 2);
            if rho < crossPoint.rho
                crossPoint.x = x0;
                crossPoint.y = y0;
                crossPoint.rho = rho;
            end
        end
    end
    
    fun = @(x) fy(x) - fy6(x);
    x0 = fsolve(fun, 0);
    if x0 >= 10 && x0 <= 12 % 激光所在直线有交点
        y0 = fy(x0);
        theta_ = atan2(y0-Pos.y, x0-Pos.x);
        if theta_ >= pi
            theta_ = theta_ - 2*pi;
        elseif theta_ <= -pi
            theta_ = theta_ + 2*pi;
        end
        if abs(theta_w - theta_) < 0.01 % 激光所在射线有交点
            rho = power(x0-Pos.x, 2) + power(y0-Pos.y, 2);
            if rho < crossPoint.rho
                crossPoint.x = x0;
                crossPoint.y = y0;
                crossPoint.rho = rho;
            end
        end
    end
    
    fun = @(x) fy(x) - fy7(x);
    x0 = fsolve(fun, 0);
    if x0 >= 12 && x0 <= 16 % 激光所在直线有交点
        y0 = fy(x0);
        theta_ = atan2(y0-Pos.y, x0-Pos.x);
        if theta_ >= pi
            theta_ = theta_ - 2*pi;
        elseif theta_ <= -pi
            theta_ = theta_ + 2*pi;
        end
        if abs(theta_w - theta_) < 0.01 % 激光所在射线有交点
            rho = power(x0-Pos.x, 2) + power(y0-Pos.y, 2);
            if rho < crossPoint.rho
                crossPoint.x = x0;
                crossPoint.y = y0;
                crossPoint.rho = rho;
            end
        end
    end
    
    % 圆，用二次方程解
    a = k * k + 1;
    b = 2*k*Pos.y - 2*k*k*Pos.x -34 - 26*k;
    c = Pos.y*Pos.y - 2*k*Pos.x*Pos.y + k*k*Pos.x*Pos.x - 26*Pos.y + 26*k*Pos.x + 454;
    delta = b * b - 4 * a * c;
    if delta >= 0
        x1 = (-b + sqrt(delta)) / (2 * a);
        y1 = fy(x1);
        theta_ = atan2(y1-Pos.y, x1-Pos.x);
        if theta_ >= pi
            theta_ = theta_ - 2*pi;
        elseif theta_ <= -pi
            theta_ = theta_ + 2*pi;
        end
        if abs(theta_w - theta_) < 0.01 % 激光所在射线交点
            rho = power(x1-Pos.x, 2) + power(y1-Pos.y, 2);
            if rho < crossPoint.rho
                crossPoint.x = x1;
                crossPoint.y = y1;
                crossPoint.rho = rho;
            end
            if delta > 0
                x2 = (-b - sqrt(delta)) / (2 * a);
                y2 = fy(crossPoint.x);
                rho = power(x2-Pos.x, 2) + power(y2-Pos.y, 2);
                if rho < crossPoint.rho
                    crossPoint.x = x2;
                    crossPoint.y = y2;
                    crossPoint.rho = rho;
                end
            end
        end
    end
    
    % 边框
    if abs(theta_w) < 0.01 % 右
        rho = power(20-Pos.x, 2);
        if rho < crossPoint.rho
            crossPoint.x = 20;
            crossPoint.y = Pos.y;
            crossPoint.rho = rho;
        end
    elseif abs(theta_w-pi) < 0.01 || abs(theta_w+pi) < 0.01 % 左
        rho = power(Pos.x, 2);
        if rho < crossPoint.rho
            crossPoint.x = 0;
            crossPoint.y = Pos.y;
            crossPoint.rho = rho;
        end
    elseif abs(theta_w-pi/2) < 0.01 % 上
        rho = power(20-Pos.y, 2);
        if rho < crossPoint.rho
            crossPoint.x = Pos.x;
            crossPoint.y = 20;
            crossPoint.rho = rho;
        end
    elseif abs(theta_w+pi/2) < 0.01 % 下
        rho = power(Pos.y, 2);
        if rho < crossPoint.rho
            crossPoint.x = Pos.x;
            crossPoint.y = 0;
            crossPoint.rho = rho;
        end
    else % 斜着，四条边框都有交点
        x0 = 0;
        y0 = fy(x0);
        rho = power(Pos.x-x0, 2) + power(Pos.y-y0, 2);
        theta_ = atan2(y0-Pos.y, x0-Pos.x);
        if theta_ >= pi
            theta_ = theta_ - 2*pi;
        elseif theta_ <= -pi
            theta_ = theta_ + 2*pi;
        end
        if abs(theta_w - theta_) < 0.01 && rho < crossPoint.rho && y0 >= 0 && y0 <= 20 % 激光所在射线有交点
                crossPoint.x = x0;
                crossPoint.y = y0;
                crossPoint.rho = rho;
        end
        x0 = 20;
        y0 = fy(x0);
        rho = power(Pos.x-x0, 2) + power(Pos.y-y0, 2);
        theta_ = atan2(y0-Pos.y, x0-Pos.x);
        if theta_ >= pi
            theta_ = theta_ - 2*pi;
        elseif theta_ <= -pi
            theta_ = theta_ + 2*pi;
        end
        if abs(theta_w - theta_) < 0.01 && rho < crossPoint.rho && y0 >= 0 && y0 <= 20 % 激光所在射线有交点
                crossPoint.x = x0;
                crossPoint.y = y0;
                crossPoint.rho = rho;
        end
        y0 = 0;
        x0 = (y0 - Pos.y) / k + Pos.x;
        rho = power(Pos.x-x0, 2) + power(Pos.y-y0, 2);
        theta_ = atan2(y0-Pos.y, x0-Pos.x);
        if theta_ >= pi
            theta_ = theta_ - 2*pi;
        elseif theta_ <= -pi
            theta_ = theta_ + 2*pi;
        end
        if abs(theta_w - theta_) < 0.01 && rho < crossPoint.rho && x0 >= 0 && x0 <= 20 % 激光所在射线有交点
                crossPoint.x = x0;
                crossPoint.y = y0;
                crossPoint.rho = rho;
        end
        y0 = 20;
        x0 = (y0 - Pos.y) / k + Pos.x;
        rho = power(Pos.x-x0, 2) + power(Pos.y-y0, 2);
        theta_ = atan2(y0-Pos.y, x0-Pos.x);
        if theta_ >= pi
            theta_ = theta_ - 2*pi;
        elseif theta_ <= -pi
            theta_ = theta_ + 2*pi;
        end
        if abs(theta_w - theta_) < 0.01 && rho < crossPoint.rho && x0 >= 0 && x0 <= 20 % 激光所在射线有交点
                crossPoint.x = x0;
                crossPoint.y = y0;
                crossPoint.rho = rho;
        end
    end
    
end
