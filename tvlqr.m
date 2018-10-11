function [K,S,e] = tvlqr(t, theta, A, B, Q,R,N)
% Computes the optimal gain controller at a point in time.

% For now this is hard-coded as the linearization of the dynamics.

% Calculate a controller for every-time point along the nominal trajectory.
for i=1:10:length(t)
    % Fixup the A-array.
    A_local = A(theta(i))
    B_local = B() % Dummy, gives same vector every time!
    [K(i), S(i), e(i)] = lqr(A_local, B_local, Q, R, N);
end

end