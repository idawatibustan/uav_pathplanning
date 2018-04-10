
%point_nbr - OK
% max_vel - OK
%alpha_previous
%alpha
%path_length - OK

if (point_nbr < 10)
    speed = max_vel * (point_nbr/10) ;
elseif (point_nbr > path_length -10 )
    speed = max_vel * (path_length-point_nbr)/10 ;
end

angle = abs(alpha_previous - alpha) ;
speed = speed * (1-angle/(3.14/2)) ;


