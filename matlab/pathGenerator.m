clc
clearvars -except smooth_path
close all

%% Path generator script
%  Author: Erwin Dassen (EDAS)
%  Date last update: 2013-11-05

%% Description and usage
%
% This script is a general purpose multidimensional path generator
% algorithm. Given n-dimensional waypoints described as pairs of positions
% and velocities, it calculates a path meeting these waypoints and
% guaranteed to be below maximum thresholds of velocity, acceleration and
% jurk. Keep in mind that the path created is not necessarily optimal.
%
% For the more interested: the algorithm is nothing more than a 3-piece 3rd
% order spline generator. Each movement section (phase-space between
% waypoints) is calculated independently of the others and is found by
% considering three 3rd order splines that "fit" in the section. After such
% one is generated, it is cheked if it fulfills the threshold conditions.
% If not, more time is aloted to that movement section. This is repeated
% until a valid path is found or MAXCOUNT iterations have been performed
% without on solution.
%
% Usage: set up the relevant script constants in the next section either by
% editing this script directly or by creating constants of the same name in
% the workspace. Comments were added where they are defined.
%
% Output: Matrices ttime, path, velocity, acceleration, jurk. The first,
% ttime, is just the time axis as defined by the time_step constant and the
% total movement time (this is calculated during execution). The rest of
% the matrices encode column vectors with the respective values for
% position, velocity, acceleration and jurk at a given instant of time (is
% sequence with ttime).
%
% TODO: Try to improve optimization to find paths of low total curvature
% (less faults due to inertia).

%% Scripting constants

% Plotting constants
if exist('pathGen','var') ~= 1
    pathGen.lplot = true;
    pathGen.lplotvel = true;

    % The dimension of the problem (6DOF = 6).
    pathGen.dim = 2;

    % The time resolution for the output.
    pathGen.time_step = 0.05;

    % The time interval increment step size. If a path found is not valid we
    % give more time for a path and rerun the algorithm. This determines by how
    % much we increment the time budget.
    pathGen.tf_step = 0.5;

    % The maximum number of iterations.
    pathGen.MAXCOUNT = 50;

    % The velocity, acceleration and jurk constraints. Written in dim x 1
    % vectors with positive entries.
    pathGen.v_max = [1;1];
    pathGen.a_max = [8;8];
    pathGen.j_max = [100;100];

    % The waypoints in the state-space for the path. Arraged in two dim x n matrices where n is
    % the number of waypoints (n > 1). The first, array_ps contains positions
    % and the second array_vs velocities. The code tests if these already
    % exists in the workspace and uses that in that case. You can choose to
    % specidy here instead.

    max_vel = 1 ; % Maximum velocity allowed
    n = length(smooth_path); % Number of points in position vector
    dim = 2; % Number of dimensions

    pathGen.array_ps = smooth_path'; % A square movement.
    pathGen.array_vs = zeros(dim,n);
    path_length = length(pathGen.array_ps) ;
    alpha = 0 ;

    for i = 2:length(pathGen.array_ps)-1
        point_nbr = i ;
        speed = max_vel ;
        delta_y = pathGen.array_ps(2,i+1)-pathGen.array_ps(2,i);
        delta_x = pathGen.array_ps(1,i+1)-pathGen.array_ps(1,i);
        alpha_previous = alpha ;
        alpha = atan(delta_y/delta_x);
        %speed_limit ;
        pathGen.array_vs(1,i) = speed * cos(alpha);
        pathGen.array_vs(2,i) = speed * sin(alpha);
        if delta_y == 0
            if delta_x < 0
                pathGen.array_vs(1,i) = -speed;
            end
        elseif delta_x == 0
            if delta_y < 0
                pathGen.array_vs(2,i) = -speed;
            end
        end
    end
end

lplot = pathGen.lplot;
lplotvel = pathGen.lplotvel;
dim = pathGen.dim;
time_step = pathGen.time_step;
tf_step = pathGen.tf_step;
MAXCOUNT = pathGen.MAXCOUNT;
v_max = pathGen.v_max;
a_max = pathGen.a_max;
j_max = pathGen.j_max;
array_ps = pathGen.array_ps;
array_vs = pathGen.array_vs;

[mp,np] = size(array_ps);
[mv,nv] = size(array_vs);

%% Sanity checks
assert(dim > 0,'Wrong type for input: must be positive integer.');
assert(all(size([v_max a_max j_max]) == [dim 3]),'Wrong size for input: constraints should have size equal to dimension.');
assert(all(all([v_max a_max j_max] >= zeros(dim,3))),'Input must be positive.');
assert(time_step > 0,'Step out of range.');
assert(all(size(array_ps) == size(array_vs)),'Wrong size on input: array_ps and array_vs should have same size.');
assert(np > 1,'Specify at least two poits for path.');
assert(nv > 1,'Specify at least two poits for path.');
assert(all(all(abs(array_vs) <= repmat(v_max,1,np))),'Velocities out of range.');

%% System setup

% Clean up output variables
clear ttime; clear path; clear velocity; clear acceleration; clear jurk;

% Linear system functions
z = zeros(1,dim);
rhs = @(pc,pn,vc,vn) vertcat(pc,pn,z,z,vc,vn,z,z,z,z,z,z);
clear z;

lhs = @(t)[1,0,0,0,0,0,0,0,0,0,0,0; ...
    0,0,0,0,0,0,0,0,1,t,t^2,t^3; ...
    1,(1/3)*t,(1/9)*t^2,(1/27)*t^3,-1,-(1/3)*t,-(1/9)*t^2,-(1/27)*t^3,0,0,0,0; ...
    0,0,0,0,1,(2/3)*t,(4/9)*t^2,(8/27)*t^3,-1,-(2/3)*t,-(4/9)*t^2,-(8/27)*t^3; ...
    0,1,0,0,0,0,0,0,0,0,0,0; ...
    0,0,0,0,0,0,0,0,0,1,2*t,3*t^2; ...
    0,1,(2/3)*t,(1/3)*t^2,0,-1,-(2/3)*t,-(1/3)*t^2,0,0,0,0; ...
    0,0,0,0,0,1,(4/3)*t,(4/3)*t^2,0,-1,(-4/3)*t,(-4/3)*t^2; ...
    0,0,2,0,0,0,0,0,0,0,0,0; ...
    0,0,0,0,0,0,0,0,0,0,2,6*t; ...
    0,0,2,2*t,0,0,-2,-2*t,0,0,0,0; ...
    0,0,0,0,0,0,2,4*t,0,0,-2,-4*t];

% An utility index vector used below.
idx = [1 1 1 1 2 2 2 2 3 3 3 3];

% The polynomialy defined position, velocity, acceleration and jurk
% functions. Each composed of three polynomial pieces.
p = @(i,t,coef) transpose([1 t t^2 t^3]*coef(idx == i,:));
v = @(i,t,coef) transpose([0 1 2*t 3*t^2]*coef(idx == i,:));
a = @(i,t,coef) transpose([0 0 2 6*t]*coef(idx == i,:));
j = @(i,t,coef) transpose([0 0 0 6]*coef(idx == i,:));

ttime = 0; % The time axis. Has to be defined outside the for loop.
path = {array_ps(:,1)}; % Output path. We start with the first position.
velocity = {array_vs(:,1)}; % Output velocity. We start with the initial velocity.
acceleration = {zeros(dim,1)}; % Output acceleration. We start at zero.
jurk = {zeros(dim,1)}; % Output jurk. We start at zero.

%% Actual path computation

for ii = 1:(np - 1) % np - 1 path segments
    pc = array_ps(:,ii); % Current starting point.
    vc = array_vs(:,ii); % Current starting velocity.
    
    pn = array_ps(:,ii+1); % Current destination point.
    vn = array_vs(:,ii+1); % Current destination velocity.

    % As a first approximation we take an average time. This movement is
    % probably impossible. We raise this time interval by tf_step. 
    tf = max(max([abs(pn-pc)./v_max, abs(vn-vc)./a_max]));
    
    % Logical for the test if the path is valid.
    pathValid = false;
    
    % Just a counter.
    count = 0;
    
    % The righ hand side.
    b = rhs(pc',pn',vc',vn'); % A 12 x dim matrix

    %% Per segment computation    
    while all(~pathValid) && (count < MAXCOUNT)
        count = count + 1;
        
        A = lhs(tf);
        
        % The coefficients of the three polynomials
        coef = A\b; % One column per dimension.
        
        % Check if the path is valid
        
        % Jurk constraint
        if any([any(abs(j(1,(1/3)*tf,coef)) > j_max), ...
                any(abs(j(2,(2/3)*tf,coef)) > j_max), ...
                any(abs(j(3,tf,coef)) > j_max)])
            % Constraint failed.
            tf = tf + tf_step; % Allow for more time.
            continue;
        end
        
        % Acceleraiton constraint
        if any([any(abs(a(1,(1/3)*tf,coef)) > a_max), ...
                any(abs(a(2,(2/3)*tf,coef)) > a_max)])
            % Constraint failed.
            tf = tf + tf_step; % Allow for more time.
            continue;
        end

        % Velocity constraint
        % This is more difficult because the extremes of velocity are not
        % in the edges of the segments and it will vary per dimension.
        
        % The critical points of v. It is in a dim x 3 format where each
        % collumn corresponds to one of the 3 splines in this path segment.
        
        tvc = (-1/3).*[coef(3,:)./coef(4,:); ... 
            coef(7,:)./coef(8,:); ...
            coef(11,:)./coef(12,:)]';
        
        % The corresponding velocity values.
        testv = zeros(dim,3);
        for jj = 1:dim
            testv(jj,:) = [v(1,tvc(jj,1),coef(:,jj)) ...
                v(2,tvc(jj,2),coef(:,jj)) ...
                v(3,tvc(jj,3),coef(:,jj))];
        end
        
        % NaN will *not* be a problem since logical tests fail with NaN.
        if any(any(testv > repmat(v_max,1,3)))
            % Constraint failed.
            tf = tf + tf_step; % Allow for more time.
            continue;
        end

        % All constraints satisfied.
        pathValid = true;
        
        if count == MAXCOUNT
            warning('MAXCOUNT reached!');
        end
    end
    % Clean 

    %% Compute partial output
    
    % Evaluate the polynomials on the given grid.
    time_seg = time_step:time_step:tf;
    i1 = time_seg <= (1/3)*tf;
    i2 = ((1/3)*tf < time_seg) & (time_seg <= (2/3)*tf);
    i3 = (2/3)*tf < time_seg;
    l1 = length(time_seg(i1));
    l2 = length(time_seg(i2));
    l3 = length(time_seg(i3));
    tempfunp = @(i,t) p(i,t,coef);
    tempfunv = @(i,t) v(i,t,coef);
    tempfuna = @(i,t) a(i,t,coef);
    tempfunj = @(i,t) j(i,t,coef);
    

    ttime = horzcat(ttime, ttime(end) + time_seg);
    
    path = horzcat(path, ...
        arrayfun(tempfunp,ones(1,l1),time_seg(i1),'UniformOutput',false), ...
        arrayfun(tempfunp,2.*ones(1,l2),time_seg(i2),'UniformOutput',false), ...
        arrayfun(tempfunp,3.*ones(1,l3),time_seg(i3),'UniformOutput',false));
    velocity = horzcat(velocity, ...
        arrayfun(tempfunv,ones(1,l1),time_seg(i1),'UniformOutput',false), ...
        arrayfun(tempfunv,2.*ones(1,l2),time_seg(i2),'UniformOutput',false), ...
        arrayfun(tempfunv,3.*ones(1,l3),time_seg(i3),'UniformOutput',false));
    
    acceleration = horzcat(acceleration, ...
        arrayfun(tempfuna,ones(1,l1),time_seg(i1),'UniformOutput',false), ...
        arrayfun(tempfuna,2.*ones(1,l2),time_seg(i2),'UniformOutput',false), ...
        arrayfun(tempfuna,3.*ones(1,l3),time_seg(i3),'UniformOutput',false));

    jurk = horzcat(jurk, ...
        arrayfun(tempfunj,ones(1,l1),time_seg(i1),'UniformOutput',false), ...
        arrayfun(tempfunj,2.*ones(1,l2),time_seg(i2),'UniformOutput',false), ...
        arrayfun(tempfunj,3.*ones(1,l3),time_seg(i3),'UniformOutput',false));
    
end

%% Output
% Transform final output to matrices (not cells)
path = horzcat(path{:});
velocity = horzcat(velocity{:});
acceleration = horzcat(acceleration{:});
jurk = horzcat(jurk{:});

%% Display statistics
fprintf('Time: %f\n',ttime(end));
fprintf('Movepoints: %d\n',length(ttime));
fprintf('Iterations: %d\n\n',count);
fprintf('Max abs. velocity (per dim.): '); disp(abs(max(velocity,[],2)'));
fprintf('Max abs. acceleration (per dim.): '); disp(abs(max(acceleration,[],2)'));
fprintf('Max abs. jurk (per dim.): '); disp(abs(max(jurk,[],2)'));

%% Clean-up
clear idx;
clear mp;clear np;clear mv;clear nv;
clear pc;clear pn;clear vc;clear vn;
clear pathValid;
clear ii;
clear A;
clear b;
clear count;
clear tvc;
clear testv;
clear jj;
clear tf;
clear i1;clear i2;clear i3;
clear l1;clear l2;clear l3;
clear time_seg;
clear tempfunp;clear tempfunv;clear tempfuna;clear tempfunj;
clear p;clear v;clear a;clear j;
clear lhs;clear rhs;
clear coef;

%% Plots (only if dim = 2,3)

if dim == 2 && lplot
    plot_step = 1:100*time_step:length(path(1,:));
    figure; hold on;
    plot(path(1,:),path(2,:),'Color','blue');

    if lplotvel
        quiver(path(1,plot_step),path(2,plot_step),...
            velocity(1,plot_step),velocity(2,plot_step),...
            0,'Color','red');
        quiver(array_ps(1,:),array_ps(2,:),...
            array_vs(1,:),array_vs(2,:),...
            0,'Color','green');
    end
    
    title('Path');
    grid;
    clear plot_step;
    
    figure; hold on;
    plot(ttime,velocity(1,:),ttime,velocity(2,:));
    title('Velocity');
    
    figure; hold on;
    plot(ttime,acceleration(1,:),ttime,acceleration(2,:));
    title('Acceleration');
    
    figure; hold on;
    plot(ttime,jurk(1,:),ttime,jurk(2,:));
    title('Jurk');
    
elseif dim == 3 && lplot
    plot_step = 1:100*time_step:length(path(1,:));
    figure; hold on;
    plot3(path(1,:),path(2,:),path(3,:),'Color','blue');
    
    if lplotvel
        quiver3(path(1,plot_step),path(2,plot_step),path(3,plot_step),...
            velocity(1,plot_step),velocity(2,plot_step),velocity(3,plot_step),...
            0,'Color','red');
        quiver3(array_ps(1,:),array_ps(2,:),array_ps(3,:),...
            array_vs(1,:),array_vs(2,:),array_vs(3,:),...
            0,'Color','green');
    end
    
    title('Path');
    grid;
    clear plot_step;
    
    figure; hold on;
    plot(ttime,velocity(1,:),ttime,velocity(2,:),ttime,velocity(3,:));
    title('Velocity');
    
    figure; hold on;
    plot(ttime,acceleration(1,:),ttime,acceleration(2,:),...
        ttime,acceleration(3,:));
    title('Acceleration');
    
    figure; hold on;
    plot(ttime,jurk(1,:),ttime,jurk(2,:),ttime,jurk(3,:));
    title('Jurk');
    
end

%% Create timeseries object

pathGen.path = timeseries(path',ttime','IsTimeFirst',true,'Name','Position');
pathGen.path.DataInfo.Units = 'm';

pathGen.velocity = timeseries(velocity',ttime','IsTimeFirst',true,'Name','Velocity');
pathGen.velocity.DataInfo.Units = 'm/s';

pathGen.acceleration = timeseries(acceleration',ttime','IsTimeFirst',true,'Name','Acceleration');
pathGen.acceleration.DataInfo.Units = 'm/s^2';

pathGen.jurk = timeseries(jurk',ttime','IsTimeFirst',true,'Name','Jurk');
pathGen.jurk.DataInfo.Units = 'm/s^3';

%% Clean up

clear lplot;
clear lplotvel;
clear dim;
clear time_step;
clear tf_step;
clear MAXCOUNT;
clear v_max;
clear a_max;
clear j_max;
clear array_ps;
clear array_vs;

clear ttime;
clear path;
clear velocity;
clear acceleration;
clear jurk;
