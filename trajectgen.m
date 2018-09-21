% MAIN.m  --  Toy Car
%
% Dynamics:
%   A simple model of a car, where the state is its position and
%   orientation, and the control is the rate of change in steering.
%
% Objective:
%   Find the best path between two points that avoids driving on steep
%   slopes.
%

clc; clear;

xBnd = [-50,50];
yBnd = [-50,50];

startPoint = [0;0];   %Start here
finishPoint = [2;4.5];   %Finish here

uMax = 100.0;  %Max steering rate

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                      Set up function handles                            %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

problem.func.dynamics = @(t,x,u)( dynamics(x,u) );
problem.func.pathObj = @(t,x,u)( u.^2 );


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                 Set up bounds on state and control                      %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

problem.bounds.initialTime.low = 0;
problem.bounds.initialTime.upp = 0;
problem.bounds.finalTime.low = 0.1;
problem.bounds.finalTime.upp = 100;

problem.bounds.state.low = [xBnd(1); yBnd(1); -2*pi];
problem.bounds.state.upp = [xBnd(2); yBnd(2);  2*pi];

problem.bounds.initialState.low = [startPoint; -2*pi];
problem.bounds.initialState.upp = [startPoint; 2*pi];

problem.bounds.finalState.low = [finishPoint; -2*pi];
problem.bounds.finalState.upp = [finishPoint; 2*pi];

problem.bounds.control.low = -uMax;
problem.bounds.control.upp = uMax;


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                 Initialize trajectory with guess                        %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
% Car travels at a speed of one, and drives in a straight line from start
% to finish point.

del = finishPoint - startPoint;  % vector from start to finish
angle = atan2(del(2),del(1));

problem.guess.time = [0, norm(del)];   % time = distance/speed
problem.guess.state = [ [startPoint; angle], [finishPoint; angle]];
problem.guess.control = [0,0];


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                      Options for Transcription                          %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

problem.options.nlpOpt = optimset(...
    'display','iter',...
    'MaxFunEval',1e5,...
    'tolFun',1e-6);

% problem.options.method = 'hermiteSimpson';
% problem.options.hermiteSimpson.nSegment = 25;

% problem.options.method = 'gpops';

problem.options.method = 'trapezoid';
problem.options.trapezoid.nGrid = 15;

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                            Solve!                                       %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

finishPoints(1,:) = -2:1:2;
finishPoints(2,:) = repmat(10,1,5);

figure(1); clf; hold on;

for i = 1:length(finishPoints)

    problem.bounds.initialState.low = [startPoint; 0];
    problem.bounds.initialState.upp = [startPoint; 0];

    problem.bounds.finalState.low = [finishPoints(:,i); 0];
    problem.bounds.finalState.upp = [finishPoints(:,i); 0];

    soln = optimTraj(problem);


    %~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
    %                        Display the solution                             %
    %~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%


    t = linspace(soln.grid.time(1), soln.grid.time(end), 150);
    z = soln.interp.state(t);
    x = z(1,:);
    y = z(2,:);
    th = z(3,:);
    u = soln.interp.control(t);
    sol = [t;x;y;th;u];
    solutions(:,:,i) = sol;

    tGrid = soln.grid.time;
    xGrid = soln.grid.state(1,:);
    yGrid = soln.grid.state(2,:);
    thGrid = soln.grid.state(3,:);
    uGrid = soln.grid.control;

    % Plot the entire trajectory
    plot(x,y,'r-','LineWidth',3);

    % Plot the grid points:
    plot(xGrid, yGrid, 'ko','MarkerSize',5,'LineWidth',3);

    % Plot the start and end points:
    plot(x([1,end]), y([1,end]),'ks','MarkerSize',12,'LineWidth',3);

    % % Plot the state and control:
    % figure(2); clf;

    % subplot(2,2,1); hold on;
    % plot(t,x);
    % plot(tGrid,xGrid,'ko','MarkerSize',5,'LineWidth',3);
    % ylabel('x');

    % subplot(2,2,3); hold on;
    % plot(t,y);
    % plot(tGrid,yGrid,'ko','MarkerSize',5,'LineWidth',3);
    % ylabel('y');

    % subplot(2,2,2); hold on;
    % plot(t,th);
    % plot(tGrid,thGrid,'ko','MarkerSize',5,'LineWidth',3);
    % ylabel('θ');

    % subplot(2,2,4); hold on;
    % plot(tGrid,uGrid,'ko','MarkerSize',5,'LineWidth',3);
    % plot(t,u);
    % ylabel('u');
end

save trajectories.mat