clc
clear all
close all

%%
%----------------------------------------------------------------
%Paramter
%----------------------------------------------------------------
x_obstacle_1 = 1;
y_obstacle_1 = 1;

x_obstacle_2 = -1;
y_obstacle_2 = 1;

x_obstacle_3 = 0.5;
y_obstacle_3 = -0.5;

width = 6;
height = 6;
resolution = 0.01;

nodes = 500;
ConnectionDistance = 0.8;

r_constraint_circle = 0.4;
x_constraint_circle = 3;
y_constraint_circle = 4.5;

start = [1.5 1.5];
goal_1 = [3 5];
goal_2 = [4.5 1.5];

%%
%----------------------------------------------------------------
%Map Creation
%----------------------------------------------------------------

map = robotics.BinaryOccupancyGrid(width,height,1/resolution)

x_obstacle_1 = x_obstacle_1 + 3;
y_obstacle_1 = y_obstacle_1 + 3;

x_obstacle_2 = x_obstacle_2 + 3;
y_obstacle_2 = y_obstacle_2 + 3;

x_obstacle_3 = x_obstacle_3 + 3;
y_obstacle_3 = y_obstacle_3 + 3;

setOccupancy(map,[x_obstacle_1 y_obstacle_1], 1);
setOccupancy(map,[x_obstacle_2 y_obstacle_2], 1);
setOccupancy(map,[x_obstacle_3 y_obstacle_3], 1);
inflate(map,.6)

x = (0:resolution/1.1:width);
y = (0:resolution/1.1:height);

for i = 1:(1.1*width/resolution)
	for j = 1:(1.1*height/resolution)
		if sqrt((x(i)- x_constraint_circle)^2+(y(j)-y_constraint_circle-resolution/2)^2) < r_constraint_circle ...
			&& y(j) > y_constraint_circle+0.5*r_constraint_circle
			setOccupancy(map, [x(i) y(j)], 1)
		end
	end
end

%%
%------------------------------------------------------------------
%PRM
%------------------------------------------------------------------
planner = robotics.PRM(map,nodes)
planner.ConnectionDistance = ConnectionDistance;

path1 = findpath(planner,start,goal_1);
figure()
show(planner)
path2 = findpath(planner,goal_1,goal_2);
figure()
show(planner)
grid on

%%
%------------------------------------------------------------------
%Path Smoothing
%------------------------------------------------------------------

path = vertcat(start,path1(4:end-2,1:end),goal_1,path2(3:end-2,1:end),goal_2);

path_x = path(1:end,1);
path_y = path(1:end,2);


delta = 1:.1:size(path_x);
xq = spline(1:size(path_x),path_x,delta) ; % - 3.*ones(size(delta));
yq = spline(1:size(path_x),path_y,delta) ; %- 3.*ones(size(delta));
figure
hold on
plot(xq,yq);
scatter(path_x',path_y');
hold off

shape = size(xq) ;

smooth_path = horzcat(xq', yq') - 3.*ones(shape(2),2) ;

clearvars -except smooth_path
