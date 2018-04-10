function distance=point_to_line_distance(pt, v1, v2)
%Calculate distance between a point and a line in 2D or 3D.
% syntax:
% distance = point_to_line(pt, v1, v2)
% pt is a nx3 matrix with xyz coordinates for n points
% v1 and v2 are vertices on the line (each 1x3)
% d is a nx1 vector with the orthogonal distances
%
% 2D input is extended to 3D by setting all z-values to 0.
%
% The actual calculation is a slightly edit version of this line:
% distance=norm(cross(v1-v2,pt-v2))/norm(v1-v2)
% (this line only works for a single 3D point)
%
% Example input:
% v1 = [0,0,0];
% v2 = [3,0,0];
% pt = [0,5,0;0,10,0;0,15,0];
% distance = point_to_line_distance(pt, v1, v2);
%
% Compatibility:
% Matlab: should work on all releases (tested on R2017b, R2012b and R6.5)
% Octave: tested on 4.2.1
% OS:     should work cross-platform
%
% Version: 1.2
% Date:    2017-12-29
% Author:  H.J. Wisselink
% Email=  'h_j_wisselink*alumnus_utwente_nl';
% Real_email = regexprep(Email,{'*','_'},{'@','.'})


%parse inputs (changed to be compatible with R6.5)
if nargin~=3
    error('Incorrect number of inputs, expected 3.');
end
if ~isnumeric(pt) || ~any(size(pt,2)==[2 3]) || any(size(pt)==0)
    error('First input (pt) is not numeric or has an incorrect shape.')
end
if ~isnumeric(v1) || numel(v1)~=size(pt,2)
    error(['Second input (v1) is not numeric or has an incorrect size.' char(5+5),...
        'Expected 1x3 or 1x2, which should match the first input.'])
end
if ~isnumeric(v2) || numel(v2)~=size(pt,2)
    error(['Third input (v2) is not numeric or has an incorrect size.' char(5+5),...
        'Expected 1x3 or 1x2, which should match the first input.'])
end

%prepare inputs
v1=v1(:)';%force 1x3 or 1x2
if length(v1)==2,v1(3)=0;end%extend 1x2 to 1x3 if needed
v2=v2(:)';%force 1x3 or 1x2
if length(v2)==2,v2(3)=0;end%extend 1x2 to 1x3 if needed
v1_ = repmat(v1,size(pt,1),1) ;
v2_ = repmat(v2,size(pt,1),1) ;

%actual calculation
a = v1_ - v2_;
b = pt - v2_;
distance = sqrt(sum(cross(a,b,2).^2,2)) ./ sqrt(sum(a.^2,2));
%this is equivalent to the following line for a single point
%distance=norm(cross(v1-v2,pt-v2))/norm(v1-v2)
end