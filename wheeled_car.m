function state= wheeled_car(state,input,T,p)
%{
Simulate the robot

:vector state: current state
:vector input: applied input
:float T: Time of simlation
:float p: robot's radius
:matrix return: new state
%}

odefun_autonomus = @(t,state) wheeled_car_derivative(state,input,p);
[~,state_vec] = ode45(odefun_autonomus, [0,T], state);
state= state_vec(end,:)';


end

