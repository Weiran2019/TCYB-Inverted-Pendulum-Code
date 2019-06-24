clear all;
clc;

%%Load the data of computational time, error of cart position, error of
%%cart velocity, error of pendulum angle and error of pendulum angular
%%velocity.
load .\Data_statics\ct3.mat;
load .\Data_statics\ecp3.mat;
load .\Data_statics\ecv3.mat;
load .\Data_statics\epa3.mat;
load .\Data_statics\epav3.mat;

%%Plot the statics of computational time, error of cart position, error of
%%cart velocity, error of pendulum angle and error of pendulum angular
%%velocity in figures 6,7,8,9,10.
figure(6);
hist(ct3);
h = findobj(gca,'Type','patch');
h.FaceColor = [0.63137 0.14902 0.59608];
l1=xlabel('Image processing computational time (ms)');
l2=ylabel('Image frame number');
set(l1,'Interpreter','latex');
set(l2,'Interpreter','latex');

figure(7);
hist(ecp3);
h = findobj(gca,'Type','patch');
h.FaceColor = [0.1961 0.3216 0.6431];
l1=xlabel('Errors of cart position (m)');
l2=ylabel('Image frame number');
set(gca,'ylim',[0,600]);
set(l1,'Interpreter','latex');
set(l2,'Interpreter','latex');
[counters,centers] = hist(ecp3);

figure(8);
hist(ecv3);
h = findobj(gca,'Type','patch');
h.FaceColor = [0.9882 0.8431 0.0471];
l1=xlabel('Errors of cart velocity (m/s)');
l2=ylabel('Image frame number');
set(gca,'ylim',[0,600]);
set(l1,'Interpreter','latex');
set(l2,'Interpreter','latex');

figure(9);
hist(epa3);
h = findobj(gca,'Type','patch');
h.FaceColor = [0.6941 0.8902 0.9765];
l1=xlabel('Errors of pendulum angle (rad)');
l2=ylabel('Image frame number');
set(l1,'Interpreter','latex');
set(l2,'Interpreter','latex');

figure(10);
hist(epav3);
h = findobj(gca,'Type','patch');
h.FaceColor = [0.2157 0.4392 0.3333];
l1=xlabel('Errors of pendulum angular velocity (rad/s)');
l2=ylabel('Image frame number');
set(l1,'Interpreter','latex');
set(l2,'Interpreter','latex');
