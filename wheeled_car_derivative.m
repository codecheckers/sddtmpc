function der=wheeled_car_derivative(state,input,p)
%{
Diffrential model of wheeled robot
:vector state: state
:vector input: applied input
:float p: robot's radius
:vector return: derivatives
%}
    der=zeros(3,1);
    der(1)=input(1)*cos(state(3))-input(2)*p*sin(state(3));
    der(2)=input(1)*sin(state(3))+input(2)*p*cos(state(3));
    der(3)=input(2);
end