function plot_circle(x,y,r,color)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
t = 0:0.01:2*pi;
x_vec = x+r*cos(t);
y_vec = y+r*sin(t);
plot(x_vec,y_vec,color);
end