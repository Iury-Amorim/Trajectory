% ### tentantiva c�lculo n�o linear
% ##track1 = Q_pd(i,1); %rho
% ##track2 = Q_pd(i,2); %theta1
% ##track3 = Q_pd(i,3); %theta2
% ##track4 = Q_pd(i,4); %theta3

function F = getTrajectory(tr)
global track1 track2 track3 track4 
l1=12.8;
l2=12.8;
l3 = 8;
  
r = sqrt(tr(1)^2 + tr(3)^2);
phi = real(-atan2(tr(2),r));
beta = real(atan2(tr(2)-l3*sin(phi),r - l3*cos(phi)));
qsi = real(acos(((r - l3*cos(phi))^2 + (tr(2) - l3*sin(phi))^2 + l1^2 - l2^2)/(2*l1*sqrt((r - l3*cos((phi)))^2 + (tr(2) - l3*sin(phi))^2))));

F(1) = real(atan2(tr(3),tr(1)))-pi/2 - track1;
F(2) = (beta + qsi)-pi/2 - track2;
F(3) = real(-acos(((r - l3*cos(phi))^2 + (tr(2) - l3*sin(phi))^2 - l1^2 - l2^2)/(2*l1*l2)))- track3;
F(4) = -(phi - track2-pi/2- track3) - track4;
end%function

##function F = getTrajectory(tr)
##global track1 track2 track3 track4
##l1=12;
##l2=12;
##l3 = 7.9381;
##r = sqrt(tr(1)^2 + tr(3)^2);
##phi = real(-atan2(tr(2),r));
##beta = real(atan2(tr(2)-l3*sin(phi),r - l3*cos(phi)));
##qsi = real(acos(((r - l3*cos(phi))^2 + (tr(2) - l3*sin(phi))^2 + l1^2 - l2^2)/(2*l1*sqrt((r - l3*cos((phi)))^2 + (tr(2) - l3*sin(phi))^2))));
##
##F(1) = real(atan2(tr(3),tr(1)))-pi/2 - track1;
##F(2) = (beta + qsi)-pi/2 - track2;
##F(3) = real(-acos(((r - l3*cos(phi))^2 + (tr(2) - l3*sin(phi))^2 - l1^2 - l2^2)/(2*l1*l2)))- track3;
##F(4) = (phi - track2-pi/2- track3) - track4;
##end%function