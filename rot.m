function out = rot(theta,p)
%{
Construct transformation matrix M (from [1]). If p = 1, it constructs rotation matrix

:float theta: direction
:float p: robot's radius
:matrix return: 2d matrix M
%}
    out = [cos(theta),-p*sin(theta);
           sin(theta),p*cos(theta)];
end