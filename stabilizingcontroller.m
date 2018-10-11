function u = stabilizingcontroller(t,x,y,theta,xFit,yFit,thetaFit,uFit,kxFit,kyFit,kthetaFit)

%This function is a controller that is used to stabilize the system to a
%nominal trajectory.

xRef = polyval(xFit,t);
yRef = polyval(yFit,t);
thetaRef = polyval(thetaFit, t);
uRef = polyval(uFit,t);
Kx = polyval(kxFit,t);
Ky = polyval(kyFit,t);
Ktheta = polyval(kthetaFit,t);

u = uRef - Kx.*(x-xRef) - Ky.*(y-yRef) - Ktheta.*(theta-thetaRef);

end