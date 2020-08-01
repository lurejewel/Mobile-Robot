function MapCloud = CreateMap_SmallVersion()

% figure, hold on
% % 长方形
% plot([3,5], [12,10], 'r-');
% plot([3,8], [12,17], 'r-');
% plot([5,10], [10,15], 'r-');
% plot([8,10], [17,15], 'r-');
% % 三角
% plot([10,12], [5,9], 'r-');
% plot([10,16], [5,5], 'r-');
% plot([12,16], [9,5], 'r-');
% % 圆形
% for i = 1:628
%     plot(17+2*cos(0.01*i), 13+2*sin(0.01*i), 'r.');
% end
% % 边界
% plot([0,20], [0,0], 'r-');
% plot([0,0], [0,20], 'r-');
% plot([20,20], [0,20], 'r-');
% plot([0,20], [20,20], 'r-');

MapCloud = []; % 储存所有位置点云数据
% 边界
for i = 0:40:400
    point.x = i / 20;
    point.y = 0;
    MapCloud = [MapCloud, point];
    point.x = 0;
    point.y = i / 20;
    MapCloud = [MapCloud, point];
    point.x = i / 20;
    point.y = 20;
    MapCloud = [MapCloud, point];
    point.x = 20;
    point.y = i / 20;
    MapCloud = [MapCloud, point];
end
% 长方形
for i = 0:20:40
    point.x = 3 + i / 20;
    point.y = 12 - i / 20;
    MapCloud = [MapCloud, point];
    point.x = 8 + i / 20;
    point.y = 17 - i / 20;
    MapCloud = [MapCloud, point];
end
for i = 0:20:100
    point.x = 5 + i / 20;
    point.y = 10 + i / 20;
    MapCloud = [MapCloud, point];
    point.x = 3 + i / 20;
    point.y = 12 + i / 20;
    MapCloud = [MapCloud, point];
end
% 三角形
for i = 0:20:120
    point.x = 10 + i / 20;
    point.y = 5;
    MapCloud = [MapCloud, point];
    point.x = 10 + i / 20;
    if i <= 40
        point.y = 2 * point.x - 15;
    else
        point.y = -point.x + 21;
    end
    MapCloud = [MapCloud, point];
end
% 圆形
for i = 1:30:314
    point.x = 17 + 2 * cos(0.02*i);
    point.y = 13 + 2 * sin(0.02*i);
    MapCloud = [MapCloud, point];
end

% % 展示点云图像
% X = [];
% Y = [];
% for i = 1 : length(MapCloud)
%     X = [X MapCloud(i).x];
%     Y = [Y MapCloud(i).y];
% end
% figure, plot(X, Y, 'ro');

end
