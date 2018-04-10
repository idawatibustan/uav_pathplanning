clc
clearvars -except path
close all

%%
%----------------------------------------------------------------
%Path view plot
%----------------------------------------------------------------

figure
hold on
scatter(path(:,1), path(:,2));
hold off

%%
%----------------------------------------------------------------
%Generate ts, total_time, X (constraints)
%----------------------------------------------------------------

activepath = path;
[ts, total_time] = generate_ts(activepath);
X = traj_opt7(activepath, total_time, ts);

%%
%----------------------------------------------------------------
%Parameters
%----------------------------------------------------------------
tstep = 0.05;
time      = 0;
max_iter = total_time / tstep;

%%
%----------------------------------------------------------------
%Generate pos, vel, acc
%----------------------------------------------------------------
for iter = 1:max_iter
    t = time;
    
    if t >= total_time
        pos = p(end,:);
        vel = [0;0;0];
        acc = [0;0;0];
    else
        k = find(ts<=t);
        k = k(end);
        pos = [t^7, t^6, t^5, t^4, t^3, t^2, t, 1]*X(8*(k-1)+1:8*k,:);
        vel = [7*t^6, 6*t^5, 5*t^4, 4*t^3, 3*t^2, 2*t, 1, 0]*X(8*(k-1)+1:8*k,:);
        acc = [42*t^5, 30*t^4, 20*t^3, 12*t^2, 6*t, 2, 0, 0]*X(8*(k-1)+1:8*k,:);
    end
    px(iter) = pos(1) ;
    py(iter) = pos(2) ;
    vx(iter) = vel(1) ;
    vy(iter) = vel(2) ;
    ax(iter) = acc(1) ;
    ay(iter) = acc(2) ;
    tsr(iter) = time;
    time = time + tstep; % update simulation time
end

fileID = fopen('cruising.txt','wt');
fprintf(fileID,'%f %f 1 %f %f 0 %f %f 0 0 0 \r\n',[px; py; vx; vy; ax; ay]);
fclose(fileID);
%%
%----------------------------------------------------------------
%Plot graphs
%----------------------------------------------------------------
figure('Name', 'Path');
hold on
scatter(path(:,1), path(:,2));
scatter(px, py);
hold off

figure('Name', 'Position');
hold on
plot(tsr, px);
plot(tsr, py);
legend({'px', 'py'})
hold off

sum_vel = sqrt( plus( power(vx, 2), power(vy, 2) ) );
figure('Name', 'Velocity');
hold on
plot(tsr, vx);
plot(tsr, vy);
plot(tsr, sum_vel);
legend({'vx', 'vy', 'v'})
hold off

sum_acc = sqrt( plus( power(ax, 2), power(ay, 2) ) );
figure('Name', 'Acceleration');
hold on
plot(tsr, ax);
plot(tsr, ay);
plot(tsr, sum_acc);
legend({'ax', 'ay', 'a'})
hold off