clc
clear all

%% Constant matrices of state space
A=[0 0 1 0;
   0 0 0 1;
   0 0 0 0;
   0 29.4 0 0];
B=[0;0;1;3];
C=[100 0 0 0;
    0 50 0 0;
    0 0 0 0];
D=[0;0;1];
Bw=[0.0015;0.007;0.075;0.35];


%% Set the delay
d1=0.019;
d2=0.058;
tao=0.016;
d3=d2+tao;
d12=d2-d1;
d13=d3-d1;
u1=0.3;   % The derivative of image-induced delay
u2=0.7;   % The derivative of network-induced delay


%% Set the other parameters
e1=0.01;
e2=0.75;
e3=1.2;
e4=0.022;


%% Idensity matrix
I1=1;
I2=[1 0;
    0 1];


%% Initialize LMI
setlmis([]);

%% Define variables
X=lmivar(1,[4,1]);
Q1=lmivar(1,[4,1]);
Q2=lmivar(1,[4,1]);
Q3=lmivar(1,[4,1]);
Q4=lmivar(1,[4,1]);
Q5=lmivar(1,[4,1]);
Q6=lmivar(1,[4,1]);
Q7=lmivar(1,[4,1]);
Z1=lmivar(1,[4,1]);
Z2=lmivar(1,[4,1]);
Z3=lmivar(1,[4,1]);
Z4=lmivar(1,[4,1]);
Y=lmivar(2,[1,4]);

%% Add the items
lmiterm([1 1 1 X],A,1,'s');
lmiterm([1 1 1 Q1],1,1);
lmiterm([1 1 1 Q2],1,1);
lmiterm([1 1 1 Q3],1,1);
lmiterm([1 1 1 Q4],1,1);
lmiterm([1 1 1 Q5],1,1);
lmiterm([1 1 1 Q6],1,1);
lmiterm([1 1 1 Q7],1,1);
lmiterm([1 1 1 Z1],-1,1);
lmiterm([1 1 1 Z3],-1,1);
lmiterm([1 1 2 Z1],1,1);
lmiterm([1 1 5 Z3],1,1);
lmiterm([1 1 7 Y],B,1);
lmiterm([1 1 9 X],d1,A');
lmiterm([1 1 10 X],d12,A');
lmiterm([1 1 11 X],tao,A');
lmiterm([1 1 12 X],d13,A');

lmiterm([1 2 2 Q1],-1,1);
lmiterm([1 2 2 Z1],-1,1);
lmiterm([1 2 2 Z2],-1,1);
lmiterm([1 2 2 Z4],-1,1);
lmiterm([1 2 3 Z2],1,1);
lmiterm([1 2 7 Z4],1,1);

lmiterm([1 3 3 Q2],-(1-u1),1);
lmiterm([1 3 3 Z2],-2,1);
lmiterm([1 3 4 Z2],1,1);

lmiterm([1 4 4 Q3],-1,1);
lmiterm([1 4 4 Z2],-1,1);

lmiterm([1 5 5 Q4],-(1-u2),1);
lmiterm([1 5 5 Z3],-2,1);
lmiterm([1 5 6 Z3],1,1);

lmiterm([1 6 6 Q5],-1,1);
lmiterm([1 6 6 Z3],-1,1);

lmiterm([1 7 7 Z4],-2,1);
lmiterm([1 7 8 Z4],1,1);
lmiterm([1 7 9 -Y],d1,B');
lmiterm([1 7 10 -Y],d12,B');
lmiterm([1 7 11 -Y],tao,B');
lmiterm([1 7 12 -Y],d13,B');

lmiterm([1 8 8 Q7],-1,1);
lmiterm([1 8 8 Z4],-1,1);


lmiterm([1 9 9 X],-2*e1,1);
lmiterm([1 9 9 Z1],e1*e1,1);

lmiterm([1 10 10 X],-2*e2,1);
lmiterm([1 10 10 Z2],e2*e2,1);

lmiterm([1 11 11 X],-2*e3,1);
lmiterm([1 11 11 Z3],e3*e3,1);

lmiterm([1 12 12 X],-2*e4,1);
lmiterm([1 12 12 Z4],e4*e4,1);


%% Add the constraints of positive-definite matrix
lmiterm([-2 1 1 X],1,1);
lmiterm([-3 1 1 Q1],1,1);
lmiterm([-4 1 1 Q2],1,1);
lmiterm([-5 1 1 Q3],1,1);
lmiterm([-6 1 1 Q4],1,1);
lmiterm([-7 1 1 Q5],1,1);
lmiterm([-8 1 1 Q6],1,1);
lmiterm([-9 1 1 Q7],1,1);
lmiterm([-10 1 1 Z1],1,1);
lmiterm([-11 1 1 Z2],1,1);
lmiterm([-12 1 1 Z3],1,1);
lmiterm([-13 1 1 Z4],1,1);

%% Obtain the description of the LMI system
lmisys=getlmis;

%%Solve the LMI
[tmin,xfeas] = feasp(lmisys);

Xmat=dec2mat(lmisys,xfeas,X);
Q1mat=dec2mat(lmisys,xfeas,Q1);
Q2mat=dec2mat(lmisys,xfeas,Q2);
Q3mat=dec2mat(lmisys,xfeas,Q3);
Q4mat=dec2mat(lmisys,xfeas,Q4);
Q5mat=dec2mat(lmisys,xfeas,Q5);
Q6mat=dec2mat(lmisys,xfeas,Q6);
Q7mat=dec2mat(lmisys,xfeas,Q7);
Z1mat=dec2mat(lmisys,xfeas,Z1);
Z2mat=dec2mat(lmisys,xfeas,Z2);
Z3mat=dec2mat(lmisys,xfeas,Z3);
Z4mat=dec2mat(lmisys,xfeas,Z4);
Ymat=dec2mat(lmisys,xfeas,Y);

%% Obtain the control law K2
K=Ymat*inv(Xmat)
EX=eig(Xmat);
EQ1=eig(Q1mat);
EQ2=eig(Q2mat);
EQ3=eig(Q3mat);
EQ4=eig(Q4mat);
EQ5=eig(Q5mat);
EQ6=eig(Q6mat);
EQ7=eig(Q7mat);

%%Carry out the simulation with K2
sim('IP_with_error.slx');
TCPK2S=CPPAKS.Time;
TPAK2S=CPPAKS.Time;
CPK2S=CPPAKS.Data(:,1);
PAK2S=CPPAKS.Data(:,2);
save TCPK2S.mat TCPK2S;
save TPAK2S.mat TPAK2S;
save CPK2S.mat CPK2S;
save PAK2S.mat PAK2S;


