%%Load the data of experiments of cart positions and pendulum angles under
%%different upper bounds of image-induced delay
load .\Data_Exp\CP1.mat;
load .\Data_Exp\CP2.mat;
load .\Data_Exp\CP3.mat;
load .\Data_Exp\CP4.mat;
load .\Data_Exp\CP5.mat;
load .\Data_Exp\TCP1.mat;
load .\Data_Exp\TCP2.mat;
load .\Data_Exp\TCP3.mat;
load .\Data_Exp\TCP4.mat;
load .\Data_Exp\TCP5.mat;

load .\Data_Exp\PA1.mat;
load .\Data_Exp\PA2.mat;
load .\Data_Exp\PA3.mat;
load .\Data_Exp\PA4.mat;
load .\Data_Exp\PA5.mat;
load .\Data_Exp\TPA1.mat;
load .\Data_Exp\TPA2.mat;
load .\Data_Exp\TPA3.mat;
load .\Data_Exp\TPA4.mat;
load .\Data_Exp\TPA5.mat;

%%Find the top and bottom margin of the stable interval of image-induced 
%%delay
t=0:0.02:15;
tn=15:-0.02:0;
bot=zeros(1,751);
top=zeros(1,751);
for k=1:751
    min=CP3(k);
    if(min>CP4(k))
        min = CP4(k);
    end
    if(min > CP5(k))
        min=CP5(k);
    end
    bot(k)=min-0.01;
end
for k=1:751
    max=CP3(k);
    if(max<CP4(k))
        max = CP4(k);
    end
    if(max< CP5(k))
        max=CP5(k);
    end
    top(k)=max+0.01;
end
bot=bot(end:-1:1);

bota=zeros(1,751);
topa=zeros(1,751);
for k=1:751
    min=PA3(k);
    if(min>PA4(k))
        min = PA4(k);
    end
    if(min > PA5(k))
        min=PA5(k);
    end
    bota(k)=min-0.01;
end
for k=1:751
    max=PA3(k);
    if(max<PA4(k))
        max = PA4(k);
    end
    if(max< PA5(k))
        max=PA5(k);
    end
    topa(k)=max+0.01;
end
bota=bota(end:-1:1);

%%Plot the cart positions with different upper bounds of the image-induced
%%delay in figure 7, and plot pendulum angles with different upper bounds 
%%of the image-induced delay in figure 8.
figure(7);
f1=fill([t,tn],[top,bot],[0.85098 0.70196 1]);
set(f1,{'LineStyle'},{'none'});
hold on;
plot(TCP5,CP5,'r','lineWidth',2);
hold on;
plot(TCP4,CP4,'--g','lineWidth',2);
hold on;
plot(TCP3,CP3,':k','lineWidth',2);
hold on;
plot(TCP2,CP2,'Color',[0.92941 0.69019 0.12941],'LineStyle','-.','lineWidth',2);
hold on;
plot(TCP1,CP1,'-.b','lineWidth',2);
hold off;
set(gca,'xlim',[0,15],'xtick',0:1:15);
l1=xlabel('Time (s)');
l2=ylabel('Cart position (m)');
l3=legend('Stable $\bar d$ interval','$\bar d = 0.029s$','$\bar d = 0.032s$','$\bar d = 0.034s$','$\bar d = 0.035s$','$\bar d = 0.036s$');
set(l1,'Interpreter','latex');
set(l2,'Interpreter','latex');
set(l3,'Interpreter','latex','Location','NorthEast');

figure(8);
f1=fill([t,tn],[topa,bota],[0.85098 0.70196 1]);
set(f1,{'LineStyle'},{'none'});
hold on;
plot(TPA5,PA5,'r','lineWidth',2);
hold on;
plot(TPA4,PA4,'--g','lineWidth',2);
hold on;
plot(TPA3,PA3,':k','lineWidth',2);
hold on;
plot(TPA2,PA2,'Color',[0.92941 0.69019 0.12941],'LineStyle','-.','lineWidth',2);
hold on;
plot(TPA1,PA1,'-.b','lineWidth',2);
hold off;
set(gca,'xlim',[0,15],'xtick',0:1:15,'ylim',[-0.4,0.3]);
l1=xlabel('Time (s)');
l2=ylabel('Pendulum angle (rad)');
l3=legend('Stable $\bar d$ interval','$\bar d = 0.029s$','$\bar d = 0.032s$','$\bar d = 0.034s$','$\bar d = 0.035s$','$\bar d = 0.036s$');
set(l1,'Interpreter','latex');
set(l2,'Interpreter','latex');
set(l3,'Interpreter','latex','Location','SouthEast');
