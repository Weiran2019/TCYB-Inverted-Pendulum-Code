clear all;
clc;

%%Load the data of simulation under the controller K1
load .\Data_Sim_Exp\TCPK1S.mat;
load .\Data_Sim_Exp\TPAK1S.mat;
load .\Data_Sim_Exp\CPK1S.mat;
load .\Data_Sim_Exp\PAK1S.mat;

%%Load the data of simulation under the controller K2
load .\Data_Sim_Exp\TCPK2S.mat;
load .\Data_Sim_Exp\TPAK2S.mat;
load .\Data_Sim_Exp\CPK2S.mat;
load .\Data_Sim_Exp\PAK2S.mat;

%%Load the data of experiment under the controller K1
load .\Data_Sim_Exp\TCPK1E.mat;
load .\Data_Sim_Exp\TPAK1E.mat;
load .\Data_Sim_Exp\CPK1E.mat;
load .\Data_Sim_Exp\PAK1E.mat;

%%Plot the resluts of simulation under the controller K1 & K2, and the
%%experiment under K1. The cart positions are shown in figure 5 and the
%%pendulum angles are shown in the figure 6.
figure(5);
plot(TCPK1S,CPK1S,'--m','lineWidth',2);
hold on;
plot(TCPK2S,CPK2S,':g','lineWidth',2);
hold on;
plot(TCPK1E,CPK1E,'-b','lineWidth',2);
hold off;
set(gca,'xlim',[0,15],'xtick',[0:1:15]);
l1=xlabel('Time (s)');
l2=ylabel('(m)');
l3=legend('Cart position with $K_1$ (simulation)','Cart position with $K_2$ (simulation)','Cart position with $K_1$ (experiment)');
set(l1,'Interpreter','latex');
set(l2,'Interpreter','latex');
set(l3,'Interpreter','latex','Location','SouthEast');

figure(6);
plot(TPAK1S,PAK1S,'--m','lineWidth',2);
hold on;
plot(TPAK2S,PAK2S,':g','lineWidth',2);
hold on;
plot(TPAK1E,PAK1E,'-b','lineWidth',2);
hold off;
set(gca,'xlim',[0,15],'xtick',[0:1:15]);
l1=xlabel('Time (s)');
l2=ylabel('(rad)');
l3=legend('Pendulum angle with $K_1$ (simulation)','Pendulum angle with $K_2$ (simulation)','Pendulum angle with $K_1$ (experiment)');
set(l1,'Interpreter','latex');
set(l2,'Interpreter','latex');
set(l3,'Interpreter','latex','Location','NorthEast');
