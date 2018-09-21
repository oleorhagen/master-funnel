clear all;
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%         The system dynamics xdot = f(x,u)                 %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

syms x y theta u
xdot = [-sin(theta); cos(theta); u];
system.xdotSym = xdot;
system.stateVars = [x y theta];
system.controlVars = [u];

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%  First we need to generate our initial trajectories!      %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
load trajectories.mat % Gives us a `t,x,y,theta,u` vector.

% Store the nominal trajectories.
straight_solution = solutions(:,:,3); % The simplest case!
t_nom = straight_solution(1,:);
x_nom = straight_solution(2,:);
y_nom = straight_solution(3,:);
theta_nom = straight_solution(4,:);
u_nom = straight_solution(5,:);
nom_traj = [t_nom; x_nom; y_nom; theta_nom; u_nom];

nTime = 150;
tSol = linspace(0, t_nom(end), nTime);

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%     Store the reference trajectory                        %
%     Use polyfit to smooth the generated trajectories      %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

%We will store each trajectory as a polynomial. For low-order fits, say
%less than 10th order, Matlab's polyval and polyfit are good. For
%higher-order fit's, it is best to use different method, based on
%barycentric interpolation. Google chebyfun.

nFit = 5;  %Order of polynomial fitting
size(x_nom)
size(tSol)
xFit = polyfit(tSol, x_nom, nFit);
yFit = polyfit(tSol, y_nom, nFit);
thetaFit = polyfit(tSol, theta_nom, nFit);
uFit = polyfit(tSol, u_nom, nFit);
% Create a nominal trajectory vector out of the fitted polynomials.
nomTraj.xNom = xFit;
nomTraj.yNom = yFit;
nomTraj.thetaNom = thetaFit;
nomTraj.uNom = uFit;

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
% Then create the LQR controller around the nominal trajectory. %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

Q = eye(3);  % Running cost on state
R = 1;       % Running cost on input
F = eye(3);  % Terminal cost on state
tol = 1e-6;  % Accuracy of ricatti propagation

linSys = @(t) getLinearTrajectory(tSol, system, nomTraj);

Soln = trajectoryLQR(tSol,linSys,Q,R,F,tol)

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
% Linearize the closed loop system x_bar_dot = f_cl(t,xbar(t)) %
% This is because we need to make sure we have polynomial      %
% dynamics in our system.                                      %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

% polyNomial = taylorApproximation(xdot, [x,y,theta], [x_nom;y_nom;theta_nom;u_nom], 3);
