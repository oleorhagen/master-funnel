%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
% Initial implementation of the funnel (algorithm one) %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

yalmip('clear');

% Initialize V and rho, with V = xbar^TSxbar, and rho = exp(somethinglarge)
costPrev = inf;
converged = false;

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
% Initialize all the optimizer polynomials %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
t = sdpvar(1);


while ~converged
    % Step one:
    % Minimize the sum of the volumes at the discrete timesteps, by searching
    % for the multiplier polynomials (L,Lt,L0_i, Leps_k) and Sk while keeping
    % V and rho fixed.
    objective = sum(vol(x));
    C1 = sos(rhodot - Vdot - L*(V - rho) - Lt*(t(T-t))); % First optimization constraint.
    C2 = sos(rho(1) - V(1) - sum(L0_i*g0_i);
    C3 = sos(1 - x'*S*x - Leps_k*(rho(tk) - V(tk)));
    C = [C1, C2, C3]; % The optimization constraints combined!
    % Step two:
    % Minimize the sum of the volumes at the discrete timesteps, by searching
    % for the multiplier polynomials (V,rho,Lt,L0_i,Sk) and keeping L and
    % Leps_k fixed
    
    cost = sum(volEps);
    
    % Check if we have exceeded our tolerance:
    if (costPrev - cost)/(costPrev) < tolerance
        converged = true;
    end

end
