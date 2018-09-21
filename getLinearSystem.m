function [A,B] = getLinearSystem(system)

A_lin = jacobian(system.xdotSym, system.stateVars); %df/dx
B_lin = jacobian(system.xdotSym, system.controlVars); %df/du

A = matlabFunction(A_lin, 'Vars', {[system.stateVars]});
B = matlabFunction(B_lin, 'Vars', {[system.controlVars]});

% % Create the matrix values along the nominal trajectory.

% A = @(t) A_lin_fun([nomTraj.x_nom(t), nomTraj.y_nom(t), nomTraj.u_nom(t)])
% B = @(t) B_lin_fun([nomTraj.u_nom(t)])
end