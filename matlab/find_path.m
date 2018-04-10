clc
clear all
close all

%%
%----------------------------------------------------------------
%Drone params
%----------------------------------------------------------------
%maximum velocities
v_max_front = 3.1 ;
v_max_up = 2.2 ;
v_min_up = -0.55 ;

%maximum accelerations
acc_max_front = 2.8 ;
acc_max_up =  2 ;
acc_min_up = -0.5 ;

% maximum jerks
jerk_max_front = 7.1 ;
jerk_max_up = 5.0 ;
jerk_min_up = -5.0 ;

%%
%----------------------------------------------------------------
%Map params
%----------------------------------------------------------------
width = 6;
height = 6;
resolution = 0.01;

% prm param
nodes = 1500;
ConnectionDistance = 1;

trans = [3 3];
start = [-1.5 1.5] + trans;
goal_1 = [2 0] + trans;
goal_2 = [-1.5 -1.5] + trans;

r_constraint_circle = 0.3;
x_constraint_circle = 3;
y_constraint_circle = 4.5;

%%
%----------------------------------------------------------------
%Read obstacle data
%----------------------------------------------------------------
obs_fileID = fopen('obs','r');
obs_format = '%f %f';
obs_size = [2 inf];
obs = fscanf(obs_fileID, obs_format, obs_size);
num_obs = length(obs);
for obs_i = 1:length(obs)
    % populate obstacle
    obs(:, obs_i) = obs(:, obs_i) + 3;
    % disp(obs(:, obs_i));
end

%%
%----------------------------------------------------------------
%Map Creation
%----------------------------------------------------------------
map = robotics.BinaryOccupancyGrid(width,height,1/resolution)

% populate obstacle
for obs_i = 1:length(obs)
    setOccupancy(map, obs(:, obs_i)', 1);
end
inflate(map,.6);

% constraint_circle
x = (0:resolution/1.1:width);
y = (0:resolution/1.1:height);

 for i = 1:(1.1*width/resolution)
     for j = 1:(1.1*height/resolution)
        if sqrt((x(i)- x_constraint_circle)^2+(y(j)-y_constraint_circle-resolution/2)^2) < r_constraint_circle ...
        && y(j) > y_constraint_circle+0.5*r_constraint_circle
            setOccupancy(map, [y(j) x(i)], 1)
         end
     end
 end

figure
show(map)

%%
%------------------------------------------------------------------
%PRM
%------------------------------------------------------------------
planner = robotics.PRM(map,nodes);
planner.ConnectionDistance = ConnectionDistance;
planner

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

figure('Name', 'Path Generated');
hold on
scatter(path_x',path_y');
hold off
path = path - 3.*ones(size(path)) ;

clearvars -except path