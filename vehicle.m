
classdef vehicle
    properties
        weight
    end
    methods
        % Simple dubins car dynamics.
        function dz = dynamics(z,u)
            theta = z(3,:);
            dx = -sin(theta);
            dy = cos(theta);
            dtheta = u;
            dz = [dx;dy;dtheta];
        end
        % Get the dynamics as a polynomial.
        function p = polynomial_dynamics()
            p = 1;
        end
        function [A,B] = linearize_dynamics()
            A = [0 0 cos(theta);
                0 0 -sin(theta);
                0 0     0      ];
            B = [0;
                0;
                1];
        end
    end
end
