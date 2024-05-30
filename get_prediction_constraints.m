function [F,H] = get_prediction_constraints(A,b,horizon)
%{
This function caluclates conastraints for all future states.

:matrix A, vector: constraints such that A*x<b where x is the current state vector
:int horizon: MPC's horizon
:matrix,vector return: constraints such that F*x<H where x is the current and future state vector
%}
F=A;
for i=1:(horizon-1)
    F=blkdiag(F,A);
end
H=repmat(b,[horizon,1]);
end
