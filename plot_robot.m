function plot_robot(state,r,color)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
plot_circle(state(1),state(2),r/5,color)
state=state-[r*cos(state(3)),r*sin(state(3)),0];
plot_circle(state(1),state(2),r,color)
end