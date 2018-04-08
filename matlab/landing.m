%this is to try wther I can get to find the good profile

clc
clear all
close all

setup;

% ---- TIME ----
% t1 --> acceleration acceleration
% t2 --> cst acceleration
% t3 --> deceleration acceleration
% t4 --> acceleration deceleration 
% t5 --> cst deceleration
% t6 --> deceleration deceleration



% ---- POSITION ----
% x1 --> acceleration acceleration
% x2 --> cst acceleration
% x3 --> deceleration acceleration
% x4 --> acceleration deceleration 
% x5 --> cst deceleration
% x6 --> deceleration deceleration

% ---- ACCELERATION ----
% a1 --> acceleration acceleration
% a2 --> cst acceleration
% a3 --> deceleration acceleration
% a4 --> acceleration deceleration 
% a5 --> cst deceleration
% a6 --> deceleration deceleration

% ---- INITIAL ----
v0 = 0 ;
a0 = 0 ;
%--------------------

t = 1.2222  ;

% ---- NO CRUISING ----
t1 = acc_min_up/jerk_min ;
a1 = acc_min_up ;
v1 = 1/2 * jerk_min * t1^2 ;
x1 = 1/6 * jerk_min * t1^3  ;

t2 = 0.9 ; %this has been calculated ... not guessed
a2 = acc_min_up 
v2 = a1* t2 + v1 
x2 = 1/2 * a1 * t2^2 + v1 * t2 + x1 
t2_prime = t1 + t2 ;

t3 = abs(acc_min_up)/jerk_max ;
a3 = 0 ;
v3 =  1/2 * jerk_max *t3^2 + a2 * t3 + v2 ;
x3 = 1/6 * jerk_max *t3^3 + 1/2 * a2 * t3^2 + v2 * t3 + x2 ;
t3_prime = t2_prime + t3 ;

t4 = 1.1338 ; %this has been calculated ... not guessed
a4 = 0 ;
v4 = v3 ;
x4 = v3*t4 + x3 ;
t4_prime = t3_prime + t4 ;

t5 = sqrt(0.1) ; %this has been calculated ... not guessed
a5 = jerk_max*t5 + a4 ;
v5 = 1/2*jerk_max*t5^2 + a4*t5 + v4;
x5 = 1/6 * jerk_max * t5^3 + 1/2* a4*t5^2 + v4 * t5 + x4;
t5_prime = t4_prime + t5 ;

t6 = sqrt(0.1) ; %this has been calculated ... not guessed
a6 = jerk_min*t6 + a5;
v6 = 1/2*jerk_min*t6^2 + a5*t6 + v5 ;
x6 = 1/6 * jerk_min * t6^3 + 1/2 * a5* t6^2 + v5*t6 + x5 ;
t6_prime = t5_prime + t6;
%--------------------


%%
% plot

p = @(t,xin,vin,ain,jerk) xin + vin * t + 0.5*ain*t^2 + 1/6 * jerk * t^3;
v = @(t,vin,ain,jerk)  vin + ain*t + 1/2 * jerk * t^2;
a = @(t,ain,jerk)   ain + jerk * t;
time = 0:0.05:t6_prime ;

k = 1 ;
for i = time ;
    if (i<=t1)
        position = p(i,0,0,0,jerk_min) ;
        myspeed = v(i,0,0,jerk_min) ;
        myacceleration = a(i,0,jerk_min) ;
        myjerk = jerk_min ;
    elseif (i <= t2_prime)
        position = p(i-t1,x1,v1,a1, 0) ;
        myspeed = v(i-t1,v1,a1, 0) ;
        myacceleration = a(i-t1,a1, 0) ;
        myjerk = 0 ;
    elseif (i <= t3_prime)
        position = p(i-t2_prime,x2,v2,a2, jerk_max) ;
        myspeed = v(i-t2_prime,v2,a2, jerk_max) ;
        myacceleration = a(i-t2_prime,a2, jerk_max) ;
        myjerk = jerk_max ;
    elseif (i <= t4_prime)
        position = p(i-t3_prime,x3,v3,a3, 0) ;
        myspeed = v(i-t3_prime,v3,a3, 0) ;
        myacceleration = a(i-t3_prime,a3, 0) ;
         myjerk = 0 ;
    elseif (i <= t5_prime)
        position = p(i-t4_prime,x4,v4,a4, jerk_max) ;
        myspeed = v(i-t4_prime,v4,a4, jerk_max) ;
        myacceleration = a(i-t4_prime,a4, jerk_max) ;
        myjerk = jerk_max ;
    elseif (i <= t6_prime)
        position = p(i-t5_prime,x5,v5,a5, jerk_min) ; 
        myspeed = v(i-t5_prime,v5,a5, jerk_min) ;
        myacceleration = a(i-t5_prime,a5, jerk_min) ;
        myjerk = jerk_min ;
    else
        print ('error')
    end
    
    pos(k) = position + 1 ;
    speed(k) = myspeed ;
    acceler(k) = myacceleration ;
    jerkplot(k) =  myjerk ;
    max_pos(k) = 1 ;
    k = k+1 ;
end


fileID = fopen('landing.txt','wt');
fprintf(fileID,'-1.5 -1.5 %.3f 0 0 %.3f 0 0 %.3f 0 0 \r\n',[pos; speed; acceler]);
fclose(fileID);

figure (1) ;
plot (time, pos)
ylabel('Position (m)');
xlabel('Time (s)');
hold on ;
plot (time,speed) ;
plot (time, acceler) ;
plot (time, 1/15*jerkplot ) ;
legend('position', 'speed', 'acceleration', 'jerk')