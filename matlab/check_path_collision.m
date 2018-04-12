function [ collision ] = check_path_collision( map, pt1, pt2 )
% Check for collision of path between two points
% draw a line and check if line crossed any obstacles
% @input:       map         BinaryOccupancyGrid
%               pt1         [x y]
%               pt2         [x y]
% @output       collision   boolean
%
    collision = false;

    lxmb = @(x,mb) mb(1).*x + mb(2);
    diff = abs(pt2 - pt1) ;
    m = diff(2) / diff(1) ;
    c = pt2(2) - m * pt2(1) ;
    mb = [m c] ;

    for x = min(pt1(1), pt2(1)):max(pt1(1), pt2(1))
        y = lxmb(x , mb) ;
        obstacle = getOccupancy(map,[x y]) ;
        if obstacle == 1
            collision = true ;
            return
        end
    end
end

