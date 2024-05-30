function c = SDDTMPC_nonlcon(u,x,z,a_inv,b_inv,p,model_function,number_of_inputs,k,E,G,terminal_constraint_function,horizon,distur_function,Ts)
%{
This function checks if hard constraints of SDD-TMPC are satisfited in the chosen horizon. 

:vector inputs: traejctory of inputs
:Polyhderon error_set: The set of all errors. The code currently supports of the set with one starting point.
:vector z: nominal state
:matrix A_sys,B_sys: system/input matrices such that x{k+1}=Ax{k}+Bu{k}
:matrix Ak_sys: system matrix after applying state feedback control law
:matrix K: state feedback's gain matrix
:uint number_of_inputs: number of inputs
:Polyhderon X_set_pos: set of all feasible positions
:Polyhderon X_set_vel: set of all feasible velocities
:Polyhderon U_set: set of all feasible inputs
:Matrix/vector terminal_A,terminal_b: terminal constraints such that terminal_A*x<=terminal_b
:uint horizon: horizon of the MPC
:function distur_function: state-dependent model of distrbunce
:Polyhderon obstacle: obstacle
 :bool use_tube_center_flag: If yes, the solver will use tube's center to
propage uncertanity (see equation 16 from the paper [1]), otherwise nominal
state will be used

:float^2 return: If c<0, then the constraints are satifited. Ceq is always 0 and is added for compatiliblity with other Matlab solvers.
%}

c=horizon;
error = x-z;
x_error_set = ones(2,1)*error(1);
y_error_set = ones(2,1)*error(2);
theta_error_set = ones(2,1)*error(3);

for i=1:horizon
    v=u((number_of_inputs*(i-1)+1):number_of_inputs*i)';
    d = distur_function(theta_error_set,v)*Ts;
    % TODO testing from here
    [min_sin,max_sin,min_cos,max_cos] = get_border_sin_cos(z(3),theta_error_set);
    
    xy_error_verticies = get_xy_verticies(x_error_set,y_error_set);
    disc_system_min = c2d(ss([-k(1,1),0,0;0,-k(2,2),0;-(1/p)*[min_sin,min_cos,v(1)]],[],[],[]),Ts);
    disc_system_max = c2d(ss([-k(1,1),0,0;0,-k(2,2),0;-(1/p)*[max_sin,max_cos,v(1)]],[],[],[]),Ts);
    scaled_error_x = [min([disc_system_min.A(3,1)*(x_error_set);disc_system_max.A(3,1)*(x_error_set)]),...
        max([disc_system_min.A(3,1)*(x_error_set);disc_system_max.A(3,1)*(x_error_set)])]';
    scaled_error_y = [min([disc_system_min.A(3,2)*(y_error_set);disc_system_max.A(3,2)*(y_error_set)]),...
        max([disc_system_min.A(3,2)*(y_error_set);disc_system_max.A(3,2)*(y_error_set)])]';
    x_error_set = x_error_set*disc_system_min.A(1,1)+d(1,:)';
    y_error_set = y_error_set*disc_system_min.A(2,2)+d(2,:)';
    xy_error_verticies = (rot(z(3),p)^(-1))*(-k)*xy_error_verticies+v;
    if max(abs(theta_error_set))>pi/4
        U_set_scaling=1./(abs(cos(pi/4))+abs(sin(pi/4)));
    else
        U_set_scaling = min(1./(abs(cos(theta_error_set(1)))+abs(sin(theta_error_set(1)))),...
        1./(abs(cos(theta_error_set(2)))+abs(sin(theta_error_set(2)))));%min(b_inv/(b_inv*cos(theta_error_set(1))-a_inv*p*sin(theta_error_set(1))),...
        %a_inv/(a_inv*cos(theta_error_set(2))+b_inv*(1/p)*sin(theta_error_set(2))));  
    end
    theta_error_set = disc_system_min.A(3,3)*theta_error_set+scaled_error_x+scaled_error_y+d(3,:)';
    
    
    z = model_function(z,v);
    %Work only for theta smaller than |45| degrees
    
    failure_flag= false;
    for j=1:4
        if ~all(E*xy_error_verticies(:,j)<U_set_scaling*G)
            failure_flag=true;
            break
        end
    end

    if failure_flag
        break
    end
    c=c-1;

end

c = c+terminal_constraint_function(z);


end
