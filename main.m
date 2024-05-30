clc;clear;rng('default')

%% Parameters
% Robot's parameters
a=0.13;% max speed
p=0.0267; % robot's radius 
b=a/p; % maximum angular velocity
n=0.004; % maximum distance disturbance affecting follower 
% v_r and w_r are used to construct leader's trajectory and pd is distance
% to the leader
N=10; % horizon
max_iterations=600;

v_r= 0.015;
w_r=0.04;
pd=[-0.1;-0.1];% Initial conditions
ref_state = [0;0;pi/3];
state= [0.4;-0.2;-pi/2];
nominal_state=state;
%nominal_state=[0.2;-0.2;-pi/2];


% MPC paramters
Ts=0.2; % sampling time
Q=eye(2)*0.4; % state cost 
R=0.2; % input cost
k=1.2; %terminal law constant
K=eye(2)*2.3; % ancilary control law constant
lambda_tube= 0.6636; %input tube constant
% terminal set
A_omega = [1,1;-1,1;1,-1;-1,-1];
b_omega = ones(4,1)*0.0542*10;


%% NTMPC - preperation 
% fmincon will find such input u that E*u<G. 
% If U is a vector of u of the horizon then A_fmincon*U<b_mincon
% U also has the output state at the end.
E=[1/a,1/b;
    -1/a,1/b;
    1/a,-1/b;
    -1/a,-1/b];
G=ones(4,1)*lambda_tube;
options = optimoptions('fmincon','Algorithm','interior-point','Display','none');
[A_fmincon,b_fmincon] = get_prediction_constraints(E,G,N);
A_fmincon=[A_fmincon,zeros(N*4,3);zeros(6,N*2),[k/n,0,0;-k/n,0,0;0,k/n,0;0,-k/n,0;0,0,1/pi;0,0,-1/pi]];
b_fmincon=[b_fmincon;ones(6,1)];
b_fmincon_SDDTMPC=b_fmincon; %the output of this solver will be warm start for SDDTMPC. Because SSDTMPC
% does not allow to odify nominal state, we had to modify constraints a bit.
b_fmincon_SDDTMPC(end-5:end) =0; 

%% SDDTMPC - preperation 
dist_set_func = @(theta_err,v) disturbance_set_function(theta_err,v,p,n); 
SDDTMPC_cost_func = @(u,x,data) SDDTMPC_cost_function(u,x,data,Q,R,p,pd(1),pd(2),v_r,w_r,Ts);
fmincon_cost_func = @(u,x,data) fmincon_cost_function(u,x,data,Q,R,p,pd(1),pd(2),v_r,w_r,Ts);
SDDTMPC_model =@(x,u) discrete_model(x,u,p,Ts);


%% Simulation - preperation
% Below variables are mostly used to store history data to plot them later.
disturbance_history=zeros(2,max_iterations);
state_history = zeros(max_iterations+1,3);
state_history(1,:)=state';
state_SDDTMPC = state;
state_history_SDDTMPC  = state_history;
state_nominal_history = zeros(max_iterations+1,3);
state_nominal_history(1,:)=nominal_state';
nominal_state_SDDTMPC = nominal_state;
state_nominal_history_SDDTMPC = state_nominal_history;

state_ref_history = zeros(max_iterations+1+N,3);
state_desired_history = zeros(max_iterations+1+N,3);
error_history=zeros(max_iterations+1,1);
error_theta_history=zeros(max_iterations+1,1);
error_reference_history=zeros(max_iterations+1,1);
error_history_SDDTMPC=zeros(max_iterations+1,1);
error_theta_history_SDDTMPC=zeros(max_iterations+1,1);
error_reference_history_SDDTMPC=zeros(max_iterations+1,1);
abs_nominal_input_history = zeros(max_iterations,1);
abs_input_history = zeros(max_iterations,1);
abs_nominal_input_history_SDDTMPC = zeros(max_iterations,1);
abs_input_history_SDDTMPC = zeros(max_iterations,1);

% Construct leader's trajcetory
for i=2:(max_iterations+N+1)
    ref_state=wheeled_car(ref_state,[v_r,w_r],Ts,0);
    state_ref_history(i,:)=ref_state;
end

% Construct desired trajcetory
for i=1:(max_iterations+N+1)
    des_state=state_ref_history(i,1:2)'+rot(-state_ref_history(i,3),1)^(-1)*pd;
    state_desired_history(i,:)=[des_state;state_ref_history(i,3)]';
end


% %SDDTMPC simulation
for i=2:(max_iterations+1)
    if mod(i,15)==0
        disp(i)
    end
    data=struct("References",state_ref_history(i:(i+N),:),'PredictionHorizon',N); % Addational input of cost function
    % Get warm start
    cost_fun_fmincon = @(u) fmincon_cost_func(u,nominal_state_SDDTMPC,data);
    nlcon= @(u) fmincon_termianl_constraint(u,nominal_state_SDDTMPC,SDDTMPC_model,N,2,state_ref_history(i+N,:)',pd,A_omega,b_omega);
    [inputs,fval,exitflag,output] = fmincon(cost_fun_fmincon,zeros(2*N+3,1),A_fmincon,b_fmincon_SDDTMPC,[],[],[],[],nlcon,options);
   
    %Solve SDDTMPC
    terminal_constraint_func= @(z) SDDTMPC_terminal_constraint(z,state_ref_history(i+N,:)',pd,A_omega,b_omega);
    [inputs,fval,exitflag,message]  = SDDTMPC_solver(SDDTMPC_cost_func,inputs,state_SDDTMPC,nominal_state_SDDTMPC,...
       SDDTMPC_model,K,a,b,p,[-a;-b],[a;b],E,G/lambda_tube,terminal_constraint_func,N,dist_set_func,60,state_ref_history(i:(i+N),:),Ts);
    mv=inputs(1:2);% Get nominal input
    [dx,dy] = get_disturbance(n*Ts);  % Get disturbance
    disturbance_history(:,i-1)=[dx;dy]; % Store disturbance for the next simulation
    
    % Store data and update state
    error = state_SDDTMPC(1:2)-nominal_state_SDDTMPC(1:2); %error in position
    error_history_SDDTMPC(i)=norm(error);
    error_theta_history_SDDTMPC(i)=abs(state_SDDTMPC(3)-nominal_state_SDDTMPC(3));
    error_reference_history_SDDTMPC(i)=norm(rot(-state_SDDTMPC(3),1)*(state_ref_history(i,1:2)'-state_SDDTMPC(1:2))+...
        rot(state_ref_history(i,3)-state_SDDTMPC(3),1)*pd);
    anciallary_u=rot(state_SDDTMPC(3),p)^(-1)*(rot(nominal_state_SDDTMPC(3),p)*mv-K*error); %real input
    state_SDDTMPC=wheeled_car(state_SDDTMPC,anciallary_u,Ts,p)+[dx;dy;0]; %update state
    state_history_SDDTMPC(i,:)=state_SDDTMPC;


    nominal_state_SDDTMPC=wheeled_car(nominal_state_SDDTMPC,mv,Ts,p); %update nominal input
    state_nominal_history_SDDTMPC(i,:)=nominal_state_SDDTMPC;
    abs_nominal_input_history_SDDTMPC(i-1) = abs(mv(1))/a+abs(mv(2))/b;
    abs_input_history_SDDTMPC(i-1) = abs(anciallary_u(1))/a+abs(anciallary_u(2))/b;
end

% fmincon simulation
for i=2:(max_iterations+1)
    if mod(i,30)==0
        disp(i)
    end
    data=struct("References",state_ref_history(i:(i+N),:),'PredictionHorizon',N);
    cost_fun_fmincon = @(u) fmincon_cost_func(u,state,data);
    
    nlcon= @(u) fmincon_termianl_constraint(u,state,SDDTMPC_model,N,2,state_ref_history(i+N,:)',pd,A_omega,b_omega);
    [inputs,fval,exitflag,output] = fmincon(cost_fun_fmincon,zeros(2*N+3,1),A_fmincon,b_fmincon,[],[],[],[],nlcon,options);
    mv=inputs(1:2);
    error = -inputs(end-2:end);
    dx= disturbance_history(1,i-1);
    dy= disturbance_history(2,i-1);
    nominal_state=-error+state;

    %error = -inputs(end-2:end);
    

    error_history(i)=norm(error(1:2));
    error_theta_history(i)=abs(state(3)-nominal_state(3));
    error_reference_history(i)=norm(rot(-state(3),1)*(state_ref_history(i,1:2)'-state(1:2))+...
        rot(state_ref_history(i,3)-state(3),1)*pd);
    anciallary_u=rot(state(3),p)^(-1)*(rot(nominal_state(3),p)*mv-K*error(1:2));
    state=wheeled_car(state,anciallary_u,Ts,p)+[dx;dy;0];
    state_history(i,:)=state;


    nominal_state=wheeled_car(nominal_state,mv,Ts,p);
    state_nominal_history(i,:)=nominal_state;
    abs_nominal_input_history(i-1) = abs(mv(1))/a+abs(mv(2))/b;
    abs_input_history(i-1) = abs(anciallary_u(1))/a+abs(anciallary_u(2))/b;
end

%% Plot data
figure()
xlim([-0.8,0.4])
ylim([-0.3,0.8])
plot(state_ref_history(1:(max_iterations+1),1),state_ref_history(1:(max_iterations+1),2),'r')
hold on

plot(state_history(:,1),state_history(:,2),'b')
plot(state_history_SDDTMPC(:,1),state_history_SDDTMPC(:,2),'k')
plot(state_desired_history(2:(max_iterations+1),1),state_desired_history(2:(max_iterations+1),2),'g')

legend('Trajectory - leader','trajectory - follower (TMPC)', 'trajectory - follower (SDDTMPC)', ...
    'Desired trajectory')
title('path')
figure()

plot(error_history)
hold on
plot(error_history_SDDTMPC)
title('nominal error in position')

figure()

plot(0:0.2:(0.2*max_iterations),error_theta_history)
hold on
plot(0:0.2:(0.2*max_iterations),error_theta_history_SDDTMPC)
title('nominal error in direction')


figure()

plot(0:0.2:4.8,error_reference_history(2:26),'b')
hold on
plot(0:0.2:4.8,error_reference_history_SDDTMPC(2:26),'k')
legend('TMPC','SDDTMPC')
title('distance error to the destination')




figure()

plot(4.8:0.2:39.8,error_reference_history(26:201),'b')
hold on
plot(4.8:0.2:39.8,error_reference_history_SDDTMPC(26:201),'k')
legend('TMPC','SDDTMPC')
title('distance error to the destination')

figure()

plot(0:0.2:10,abs_nominal_input_history(1:51))
hold on
plot(0:0.2:10,abs_nominal_input_history_SDDTMPC(1:51),'k')
legend('TMPC','SDDTMPC')
title('nominal input')

figure()

plot(0:0.2:10,abs_input_history(1:51))
hold on
plot(0:0.2:10,abs_input_history_SDDTMPC(1:51),'k')
legend('TMPC','SDDTMPC')
title('input')



function x_next=discrete_model(x,u,p,Ts)
    autonomus_model =@(x) wheeled_car_derivative(x,u,p);
    x_next= RK4(x,autonomus_model,Ts);
end

function cost=myCostFunction(X,U,e,data,Q,R,p,xd,yd,vr,wr)  %x=[xe,ye,te], ue=[ve,we,xr,yr,tr]  par =[p,xd,yd,vr,wr]
    cost=0;
    for i = 1:data.PredictionHorizon
        if i==data.PredictionHorizon
            Q=0.5;
        end
        theta_e= data.References(i,3)-X(i,3);
        p_e= rot(-X(i,3),1)*(data.References(i,1:2)'-X(i,1:2)')+rot(theta_e,1)*[xd;yd];
        cost=cost+p_e'*Q*p_e;
        u_e=[-U(i,1)+(vr-xd*wr)*cos(theta_e)-xd*wr*sin(theta_e);
            -p*U(i,2)+(vr-xd*wr)*sin(theta_e)+yd*wr*cos(theta_e)];
        cost = cost + u_e'*R*u_e;

    end

end



function cost=SDDTMPC_cost_function(u,x,data,Q,R,p,xd,yd,vr,wr,Ts)
    All_U = reshape(u,length(u)/data.PredictionHorizon,data.PredictionHorizon)';
    X=zeros(data.PredictionHorizon,length(x));
    for i=1:data.PredictionHorizon
        x = discrete_model(x,All_U(i,:),p,Ts);
        X(i,:)=x;
    end
    cost=myCostFunction(X,All_U,[],data,Q,R,p,xd,yd,vr,wr);
end

function cost=fmincon_cost_function(u,x,data,Q,R,p,xd,yd,vr,wr,Ts)
    x=x+u(end-2:end);
    All_U = reshape(u(1:end-1),(length(u)-3)/data.PredictionHorizon,data.PredictionHorizon+1)';
    X=zeros(data.PredictionHorizon,length(x));
    for i=1:data.PredictionHorizon
        x = discrete_model(x,All_U(i,:),p,Ts);
        X(i,:)=x;
    end
    cost=myCostFunction(X,All_U,[],data,Q,R,p,xd,yd,vr,wr);
end

function slack=SDDTMPC_terminal_constraint(nominal_state,ref_state,pd,A_omega,b_omega)
    pe = rot(-nominal_state(3),1)*(ref_state(1:2)-nominal_state(1:2))+...
        rot(ref_state(3)-nominal_state(3),1)*pd;
    max_error= max([A_omega*pe-b_omega;0]);
    slack=1000*max_error;
end

function [c,ceq]=fmincon_termianl_constraint(inputs,nominal_state,model,horizon,number_of_inputs,ref_state,pd,A_omega,b_omega)
    nominal_state=nominal_state+inputs(end-2:end);
    for i=1:horizon
        v=inputs((number_of_inputs*(i-1)+1):number_of_inputs*i)';
        nominal_state=model(nominal_state,v);
    end
    pe = rot(-nominal_state(3),1)*(ref_state(1:2)-nominal_state(1:2))+...
        rot(ref_state(3)-nominal_state(3),1)*pd;
    c= A_omega*pe-b_omega;
    ceq=0;
end
