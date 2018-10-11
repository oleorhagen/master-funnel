
classdef trajectory
    properties
        xBnd = [-50,50];
        yBnd = [-50,50];
        startPoint = [0;0];
        finishPoint = [2;4.5];
        uMax % Needs to be given by the vehicle.
        problem % The problem to be solved.

        % Set the bounds for the optimization problem.
        problem.bounds.initalTime.low = 0;
        problem.bounds.initalTime.upp = 0;
        problem.bounds.finalTime.low = 0.1;
        problem.bounds.finalTime.upp = 100;
    end
    methods
        function r = gen_trajectory()
            r = 1;
        end
    end
end
