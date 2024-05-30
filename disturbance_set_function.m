function limits = disturbance_set_function(theta_error,v,p,distur_limit)
%{
This function caluclates disturbance given current state 
:vector theta_error: lowest and highest error in direction [rad]
:vector v: nominal inputs i.e. nominal velocity and nominal angular velocity
:float p: radius of the robot
:float distur_limit: maximum position disturbance

:matrix return: limits of disturbance
%}

limits = zeros(3,2);
limits(1:2,:)=[-distur_limit,distur_limit;-distur_limit,distur_limit];

%theta_old_limit = max(abs(theta_error))*L;
sin_limit = [-sin(theta_error(1))+theta_error(1);-sin(theta_error(2))+theta_error(2)]/p;
if v(1)>0
    sin_limit=sin_limit*v(1);
else
    sin_limit=flip(sin_limit)*v(1);
end
cos_limit = [cos(max(abs(theta_error)))-1;0];
if v(2)>0
    cos_limit=cos_limit*v(2);
else
    cos_limit=flip(cos_limit)*v(2);
end
theta_limit=sin_limit+cos_limit;
theta_limit2=[-v(1)/p*(sin(theta_error(1))-theta_error(1))+v(2)*min(cos(theta_error)-1);-...
    v(1)/p*(sin(theta_error(2))-theta_error(2))];

%limits(3,:) = [-theta_limit,theta_limit];
limits(3,:) = theta_limit';

end