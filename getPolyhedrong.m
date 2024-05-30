function [out] = getPolyhedrong(angle,a,p)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
b=ones(4,1);
A=[[cos(angle)+sin(angle);
    -cos(angle)+sin(angle);
    -cos(angle)-sin(angle);
    cos(angle)-sin(angle)],p*[-sin(angle)+cos(angle);
    sin(angle)+cos(angle);
    sin(angle)-cos(angle);
    -sin(angle)-cos(angle)]]/a;
out=Polyhedron(A,b);
end