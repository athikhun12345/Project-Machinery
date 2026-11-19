clc;
%close all;
figure('Name','Coupler Curve','NumberTitle','off');

posx = out.pickpath.Data(:,1);
posz = out.pickpath.Data(:,2);
hold on
plot(posx,posz);
