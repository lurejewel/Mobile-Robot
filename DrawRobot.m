% 作者：靳葳
% 上次编辑: 2020/7/23
% 功能： 画机器人。

function DrawRobot(Pos, color, robot_model)

x = Pos.x;
y = Pos.y;
theta = Pos.theta;
B = robot_model.length;
A = robot_model.width;
b = robot_model.axial_length;
a = robot_model.wheelspan;
wh = (B-b)/2;
wv = (A-a)/2;

if color == 'r' % 当前状态绘制
    plot([x-wh*cos(theta)-A/2*sin(theta), x-wh*cos(theta)+A/2*sin(theta)],[y-wh*sin(theta)+A/2*cos(theta), y-wh*sin(theta)-A/2*cos(theta)],'r-');
    plot([x+(wh+b)*cos(theta)-A/2*sin(theta), x+(wh+b)*cos(theta)+A/2*sin(theta)],[y+(wh+b)*sin(theta)+A/2*cos(theta), y+(wh+b)*sin(theta)-A/2*cos(theta)],'r-');
    plot([x-wh*cos(theta)-A/2*sin(theta), x+(wh+b)*cos(theta)-A/2*sin(theta)],[y-wh*sin(theta)+A/2*cos(theta), y+(wh+b)*sin(theta)+A/2*cos(theta)],'r-');
    plot([x-wh*cos(theta)+A/2*sin(theta), x+(wh+b)*cos(theta)+A/2*sin(theta)],[y-wh*sin(theta)-A/2*cos(theta), y+(wh+b)*sin(theta)-A/2*cos(theta)],'r-');
elseif color == 'w' % 上一状态擦除
    plot([x-wh*cos(theta)-A/2*sin(theta), x-wh*cos(theta)+A/2*sin(theta)],[y-wh*sin(theta)+A/2*cos(theta), y-wh*sin(theta)-A/2*cos(theta)],'w-');
    plot([x+(wh+b)*cos(theta)-A/2*sin(theta), x+(wh+b)*cos(theta)+A/2*sin(theta)],[y+(wh+b)*sin(theta)+A/2*cos(theta), y+(wh+b)*sin(theta)-A/2*cos(theta)],'w-');
    plot([x-wh*cos(theta)-A/2*sin(theta), x+(wh+b)*cos(theta)-A/2*sin(theta)],[y-wh*sin(theta)+A/2*cos(theta), y+(wh+b)*sin(theta)+A/2*cos(theta)],'w-');
    plot([x-wh*cos(theta)+A/2*sin(theta), x+(wh+b)*cos(theta)+A/2*sin(theta)],[y-wh*sin(theta)-A/2*cos(theta), y+(wh+b)*sin(theta)-A/2*cos(theta)],'w-');
elseif color == 'b' % 目标状态绘制
    plot([x-wh*cos(theta)-A/2*sin(theta), x-wh*cos(theta)+A/2*sin(theta)],[y-wh*sin(theta)+A/2*cos(theta), y-wh*sin(theta)-A/2*cos(theta)],'b-');
    plot([x+(wh+b)*cos(theta)-A/2*sin(theta), x+(wh+b)*cos(theta)+A/2*sin(theta)],[y+(wh+b)*sin(theta)+A/2*cos(theta), y+(wh+b)*sin(theta)-A/2*cos(theta)],'b-');
    plot([x-wh*cos(theta)-A/2*sin(theta), x+(wh+b)*cos(theta)-A/2*sin(theta)],[y-wh*sin(theta)+A/2*cos(theta), y+(wh+b)*sin(theta)+A/2*cos(theta)],'b-');
    plot([x-wh*cos(theta)+A/2*sin(theta), x+(wh+b)*cos(theta)+A/2*sin(theta)],[y-wh*sin(theta)-A/2*cos(theta), y+(wh+b)*sin(theta)-A/2*cos(theta)],'b-');

end

end
