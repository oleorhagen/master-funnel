function poly = taylorApproximation(f, syms, nominal_trajectory, degree)
% calculates the polynomial dynamics of a non polynomial system
% by taylor-expanding the trajectory around the nominal path
% to the third degree.

for i=1:length(nominal_trajectory)
  expansion_point = nominal_trajectory(i);
  poly = taylor(f, syms, 'Order', degree, 'ExpansionPoint', expansion_point);
end

end