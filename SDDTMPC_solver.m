function [inputs,fval,exitflag,message] = SDDTMPC_solver(cost_func,warm_start,x0,z0,model_function,K,a,b,p,...
    lb,ub,E,G,terminal_constraint_func,horizon,distur_function,max_iter,reference,Ts)
%{
This function finds a minimum of a chosen cost function under constraints of SDD-TMPC.  

:function cont_func: function to be minimized
:matrix warm_start: an initial swarm
:vector x0: state
:vector z: nominal state
:matrix A_sys,B_sys: system/input matrices such that x{k+1}=Ax{k}+Bu{k}
:matrix Ak_sys: system matrix after applying state feedback control law
:matrix K: state feedback's gain matrix
:vector^2 lb/ub: minimum/maximum inputs
:Polyhderon obstacle: obstacle
:Polyhderon X_set_pos: set of all feasible positions
:Polyhderon X_set_vel: set of all feasible velocities
:Polyhderon U_set: set of all feasible inputs
:Matrix/vector terminal_A,terminal_b: terminal constraints such that terminal_A*x<=terminal_b
:uint horizon: horizon of the MPC
:function distur_function: state-dependent model of distrbunce
:float simulation_tresh: if current cost is lower, the optimization will be terminated
:int max_iter: the optimization will be stopped if number of iterations exceed this value.
:bool use_tube_center_flag: If yes, the solver will use tube's center to
propage uncertanity (see equation 16 from the paper [1]), otherwise nominal
state will be used
 
:return: 
	:vector inputs: optimal inputs
	:float fval: optimal cost
	:int exitflag: reason why optimization stopped (read https://nl.mathworks.com/help/gads/particleswarm.html to know more)
	:struct massege: addational information about optimization (read https://nl.mathworks.com/help/gads/particleswarm.html to know more)
	
%}
InitialSwarmMatrix=warm_start(1:horizon*2)';
number_of_inputs = length(lb);
lb= repmat(lb,[horizon,1]);
ub= repmat(ub,[horizon,1]);
nvars = number_of_inputs*horizon;

data=struct("References",reference,'PredictionHorizon',horizon);

cost_func_tranposed = @(u) cost_func(u',z0,data)+10^10*SDDTMPC_nonlcon(u,x0,z0,...
    inv(a),inv(b),p,model_function,2,K,E,G,terminal_constraint_func,horizon,distur_function,Ts);


options = optimoptions('particleswarm','SwarmSize',50,'MaxIterations',max_iter,...'InitialSwarmMatrix',warm_start',
    'Display','iter','DisplayInterval',10,'MaxStallIterations',max_iter, ...
    'InitialSwarmMatrix',InitialSwarmMatrix);

[inputs,fval,exitflag,message]=particleswarm(cost_func_tranposed,nvars,lb,ub,options);
inputs=inputs';

end

