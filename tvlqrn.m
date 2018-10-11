function [A_lin_fun] = tvlqrn(sys, lqrParams, nomTraj)
% sys is the symbolic representation of the system dynamics.
% lqrparams holds the parameters needed for the lqr, e.g Q, R, etc.
% nomTraj (x0, u0).

% A_lin = jacobian(sys.xdot_sym, sys.stateVars) %df/dx
% B_lin = jacobian(sys.xdot_sym, sys.controlVars) %df/du

% A_lin_fun = matlabFunction(A_lin, 'Vars', {[sys.stateVars]})
% B_lin_fun = matlabFunction(B_lin, 'Vars', {[sys.controlVars]})

% % Create the matrix values along the nominal trajectory.

% A_lin_t = @(t) A_lin_fun([nomTraj.x_nom(t), nomTraj.y_nom(t), nomTraj.u_nom(t)])
% B_lin_t = @(t) B_lin_fun([nomTraj.u_nom(t)])

% % Solve one instance at time zero.
% [K, S, e] = lqr(A_lin_t(1), B_lin_t(1), lqrParams.Q, lqrParams.R)

nState = size(Q,1);
nInput = size(R,1);

userFun = @(t,z)rhs(t,z,linSys,Q,R,nState);
z0 = reshape(F,nState*nState,1);
tSpan = [t(end),t(1)];

options = odeset();
options.RelTol = tol;
options.AbsTol = tol;
sol = ode45(userFun,tSpan,z0);
z = deval(sol,t);

nSoln = length(t);
Soln(nSoln).t = 0;
Soln(nSoln).K = zeros(nState,nInput);
Soln(nSoln).S = zeros(nState,nState);
Soln(nSoln).E = zeros(nState,1);

for idx=1:nSoln
    i = nSoln-idx+1;
    zNow = z(:,i);
    tNow = t(i);
    S = reshape(zNow,nState,nState);
    [A,B] = linSys(tNow);
    K = R\(B'*S);
    Soln(i).t = tNow;
    Soln(i).K = K;
    Soln(i).S = S;
    Soln(i).E = eig(A-B*K);
end

end

function dz = rhs(t,z,linSys,Q,R,nState)
P = reshape(z,nState,nState);
[A,B] = linSys(t);
dP = ricatti(A,B,Q,R,P);
dz = reshape(dP,nState*nState,1);
end

function [dP, K] = ricatti(A,B,Q,R,P)
K = R\B'*P;
dP = -(A'*P + P*A - P*B*K + Q);
end