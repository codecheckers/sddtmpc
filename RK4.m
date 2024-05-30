function x=RK4(x0,model,Ts)
    %{
Runge-kutta 4 algorithm
:vector x0: current state
:function model: continous evolution function
:float Ts: sampling time

:matrix return: the next state
%}
    k1=Ts*model(x0);
    k2=Ts*model(x0+k1/2);
    k3=Ts*model(x0+k2/2);
    k4=Ts*model(x0+k3);
    x=x0+(k1+2*k2+2*k3+k4)/6;

end