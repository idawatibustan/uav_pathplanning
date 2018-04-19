clc
close all
clearvars -except map_check path obs num_obs ptb_idx

%run "find_path before and we will have the following variables:

% % % path : is the path generated
% % % obs : is a vector containing the obstacles
% % % num_obs : is the number of obstacles (from 1 to five)
% % % ptb_idx : is the index of point b in the path

%IMPORTANT speed in generate ts
speed_ts = 0.5 ;
delta_vb = 0.03 ;

% collision check obstacle inflation
coll_path_dist_threshold = .25;

%param to draw circles
ang = 0:0.01:2*pi ;
r_ext = 0.7 ; % 0.7 if we use the line
r_int = 0.6 ;
xp_ext=r_ext*cos(ang);
yp_ext=r_ext*sin(ang);
xp_int=r_int*cos(ang);
yp_int=r_int*sin(ang);

%parameters on speed
max_vel = 1 ;
min_vel = -1 ;
max_a = 2.8 ;
max_j = 7.1 ;

%%
%----------------------------------------------------------------
%Path view plot
%----------------------------------------------------------------

figure
hold on
scatter(path(:,1), path(:,2));
 for i = 1: num_obs
    plot(obs(1,i)+xp_ext,obs(2,i)+yp_ext,'--k');
    plot(obs(1,i)+xp_int,obs(2,i)+yp_int,'-r');
end;
axis([0 6 0 6])
pbaspect([1 1 1])
print('initial_path','-dpng');
hold off ;

%%
%----------------------------------------------------------------
%Generate ts, total_time, X (constraints)
%----------------------------------------------------------------

activepath = path;
[ts, total_time] = generate_ts(activepath, speed_ts)
recompute_path_velocity = 0 ;
recompute_path_jerk = 0 ;
recompute_path_ptb = 0 ;
recompute_path_ptb_slow = 0 ;
not_finish = true ;
update_all_speed = true ;

%%
%----------------------------------------------------------------
%Generate the path
%----------------------------------------------------------------

midpt = @(pt1, pt2) (pt1+pt2)/2 ;

while (not_finish)
    if (total_time > 200)
        fprintf('there has been an error when we tried to guratee max speed/jerk \n')
        fprintf('You have to decrease the Speed in "generate_ts" \n \n');
        break ;
    end

    if (update_all_speed)
        [ts, total_time] = generate_ts(activepath, speed_ts)
    end
    X = traj_opt7(activepath, total_time, ts) ;

    %Parameters
    tstep = 0.05;
    time      = 0;
    max_iter = round(total_time / tstep);
    firstk = true ;

    %Generate pos, vel, acc
    for iter = 1:max_iter
        t = time;

        if t >= total_time
            pos = p(end,:);
            vel = [0;0;0];
            acc = [0;0;0];
        else
            k = find(ts<=t);
            k = k(end); % index of time segment (do +1 for the time in the seg)

            pos = [t^7, t^6, t^5, t^4, t^3, t^2, t, 1]*X(8*(k-1)+1:8*k,:);
            vel = [7*t^6, 6*t^5, 5*t^4, 4*t^3, 3*t^2, 2*t, 1, 0]*X(8*(k-1)+1:8*k,:);
            acc = [42*t^5, 30*t^4, 20*t^3, 12*t^2, 6*t, 2, 0, 0]*X(8*(k-1)+1:8*k,:);
            jerk = [210*t^4, 120*t^3, 60*t^2, 24*t, 6, 0, 0, 0]*X(8*(k-1)+1:8*k,:);
        end

        %constraint for point B
        if (k == ptb_idx && firstk)
            firstk=false ;
            if (sqrt(power(vel(1),2) + power(vel(2),2)) > 0.3 + delta_vb)
                %decrease time interval before
                speed_ts = speed_ts*0.975 ;
                recompute_path_ptb = recompute_path_ptb + 1 ;
                update_all_speed = true ;
                break
            elseif (sqrt(power(vel(1),2) + power(vel(2),2)) < 0.3 - delta_vb)
                %decrease time interval before
                speed_ts = speed_ts*1.025 ;
                max_vel = max_vel*1.05 ; %otherwise unstable
                recompute_path_ptb_slow = recompute_path_ptb_slow + 1 ;
                update_all_speed = true ;
                break
            end
        end

        % ATTENTION the maximum speed (max_vel) we set here influences the speed we can
        % allow in generate_ts (max speed in generate_ts = 1/2 max_vel)
        % We decrease the speed by letting more time for the interval
        if (sqrt(power(vel(1),2) + power(vel(2),2)) > max_vel)
            % 10% more
            for j = k+1:length(ts)
                ts(j) = ts(j) + (ts(k+1)-ts(k))*0.05 ;
            end
            update_all_speed = false ;
            total_time = ts(length(ts)) ;
            recompute_path_velocity = recompute_path_velocity + 1  % count how many time have recompute
            break
        end
        
        % check for collision with position
        if getOccupancy(map_check, pos)
            disp('collision');
            if ( hypot(activepath(k, :), activepath(k+1, :)) > coll_path_dist_threshold )
                new_path = midpt( activepath(k, :), activepath(k+1, :) );
                activepath = [ activepath(1:k, :); new_path; activepath(k+1:end,:) ] ;
                update_all_speed = true ;
                break;
            end
        end

        px(iter) = pos(1) ;
        py(iter) = pos(2) ;
        vx(iter) = vel(1) ;
        vy(iter) = vel(2) ;
        ax(iter) = acc(1) ;
        ay(iter) = acc(2) ;
        tsr(iter) = time;
        jx(iter) = jerk(1) ;
        jy(iter) = jerk(2) ;
        time = time + tstep; % update simulation time
        if (iter == max_iter)
            not_finish = false ;
        end
    end
end


%%
%----------------------------------------------------------------
%Plot graphs
%----------------------------------------------------------------

figure('Name', 'Final Path');
hold on
scatter(activepath(:,1), activepath(:,2));
 for i = 1: num_obs
    plot(obs(1,i)+xp_ext,obs(2,i)+yp_ext,'--k');
    plot(obs(1,i)+xp_int,obs(2,i)+yp_int,'-r');
end;
axis([0 6 0 6])
pbaspect([1 1 1])
print('final_path','-dpng');
hold off ;

% figure 2
figure('Name', 'Trajectory');
hold on
scatter(path(:,1), path(:,2));
scatter(px, py);
hold on ;
 for i = 1: num_obs
    plot(obs(1,i)+xp_ext,obs(2,i)+yp_ext,'--k');
    plot(obs(1,i)+xp_int,obs(2,i)+yp_int,'-r');
end;
axis([0 6 0 6])
pbaspect([1 1 1])
print('final_trajectory','-dpng');
hold off ;

figure('Name', 'Position');
hold on
plot(tsr, px);
plot(tsr, py);
%points path
plot (ts, 3*ones(length(ts),1),'x')
plot (ts(ptb_idx), 3*ones(1,1),'xk','lineWidth',3)
legend({'px', 'py'})
print('graph1_pos','-dpng');
hold off

%%%%%% VELOCOTY PLOT %%%%%%%%%%%%%%%%%%%%%%%%%%%
sum_vel = sqrt( plus( power(vx, 2), power(vy, 2) ) );
figure('Name', 'Velocity');
hold on
plot(tsr, vx);
plot(tsr, vy);
plot(tsr, sum_vel);
%uper and lower bound
plot (tsr, ones(length(tsr),1),'k','lineWidth',2);
plot (tsr, -ones(length(tsr),1),'k','lineWidth',2);
plot (ts, zeros(length(ts),1),'x')
plot (ts(ptb_idx), zeros(1,1),'xk','lineWidth',3)
%legend
legend({'vx', 'vy', 'v'}, 'Location', 'Southwest')
ylim([-1.1 1.1])
print('graph2_vel','-dpng');
hold off

%%%%%%% ACCELERATION PLOT %%%%%%%%%%%%%%%%%%%%%%%%
sum_acc = sqrt( plus( power(ax, 2), power(ay, 2) ) );
figure('Name', 'Acceleration');
hold on
plot(tsr, ax);
plot(tsr, ay);
plot(tsr, sum_acc);
%uper and lower bound
plot (tsr, 2.8.*ones(length(tsr),1),'k','lineWidth',2);
plot (tsr, -2.8.*ones(length(tsr),1),'k','lineWidth',2);
plot (ts, zeros(length(ts),1),'x')
plot (ts(ptb_idx), zeros(1,1),'xk','lineWidth',3)
%legend
legend({'ax', 'ay', 'a'})
print('graph3_acc','-dpng');
hold off

%%%%%%%%% JERK PLOT %%%%%%%%%%%%%%%%%%%%%%%%
sum_jerk = sqrt( plus( power(jx, 2), power(jy, 2) ) );
figure('Name', 'jerk');
hold on
plot(tsr, jx);
plot(tsr, jy);
plot(tsr, sum_jerk);
%uper and lower bound
plot (tsr, 7.1.*ones(length(tsr),1),'k','lineWidth',2);
plot (tsr, -7.1.*ones(length(tsr),1),'k','lineWidth',2);
plot (ts, zeros(length(ts),1),'x')
plot (ts(ptb_idx), zeros(1,1),'xk','lineWidth',3)
%legend
legend({'jx', 'jy'})
print('graph4_jerk','-dpng');
hold off

% give performance
fprintf ('I had to recompute the path due to too large velocity %d times \n', ...
    recompute_path_velocity)
fprintf ('I had to recompute the path due to too large velocity at b %d times \n', ...
    recompute_path_ptb)
fprintf ('I had to recompute the path due to too slow velocity at b %d times \n', ...
    recompute_path_ptb_slow)

%transform back in their coordinates

%position
px = px-3*ones(1,length(px)) ;
py = py-3*ones(1,length(py)) ;
px_inter = px ;
px = py ;
py = -px_inter ;
%speed
vx_inter = vx ;
vx = vy ;
vy = -vx_inter ;
%acceleration
ax_inter = ax ;
ax = ay ;
ay = -ax_inter ;
%jerk
jx_inter = jx ;
jx = jy ;
jy = -jx_inter ;

% write result in the file
fileID = fopen('cruising.txt','wt');
fprintf(fileID,'%f %f 1 %f %f 0 %f %f 0 0 0 \r\n',[px; py; vx; vy; ax; ay]);
fclose(fileID);
