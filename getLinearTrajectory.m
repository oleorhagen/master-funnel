function [A,B] = getLinearTrajectory(t, system, nomTraj)

x = polyval(nomTraj.xNom,t);
y = polyval(nomTraj.yNom,t);
theta = polyval(nomTraj.thetaNom,t);
u = polyval(nomTraj.uNom,t);


[At,Bt] = getLinearSystem(system);

A =At([x,y,theta]);
B = Bt(u);

end