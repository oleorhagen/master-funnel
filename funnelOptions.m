
options.controller_deg = 1;

options.search_controller = true;


if options.controller_deg == 1
    options.degL1 = 2;
    options.degLu = 0;
elseif options.controller_deg == 3
    options.degL1 = 4;
    options.degLu = 2;
else
    error('controller order not handled')
end

options.max_iterations = 20;
options.converged_tol = 0.01;
options.backoff_percent = 1;

% options.degL1 = 2;
options.wmax = [0.5 0.3];
options.wmin = [-0.5 -0.3];
% options.Nmax = 0.3; %0.3;
% options.Nmin = -0.3; %-0.3;
% options.Vmax = 0.5;
% options.Vmin = -0.5;

% options.degL1 = 2;
options.degLu1 = 2;
options.degLu2 = 2;
options.degLup = 2;
options.degLum = 2;
options.degLw = 2;


options.clean_tol = 1e-6;

options.saturations = true;
options.umax = 350; % 500
options.umin = -350; % -500

options.rho0 = 1;

options.num_uncertain = 2;