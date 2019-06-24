clear all;
clc;

%%Set the parameters of the inverted pendulum
m=0.109;%Mass of the pendulum
J=0.009083;%Moment of inertia of the pendulum
l=0.25;%Length from the pivot to the center of the pendulum
g=9.8;%Acceleration of gravity

%%Caculate the constant matrix in the state space.
A=[0 0 1 0;0 0 0 1;0 0 0 0;0 l*m*g/J 0 0]
B=[0;0;1;m*l/J]

%%Set the lower bounds and upper bounds of the image-induced computational
%%delay d(t)
d1=0.019;
d2=0.089;
d12=d2-d1;

%%Set the controller
K=[2.6464 -27.0291 2.9938 -4.9654];

%% Initialize LMI
setlmis([]);

%% Define the variables
P=lmivar(1,[4,1]);
Q1=lmivar(1,[4,1]);
Q2=lmivar(1,[4,1]);
Z1=lmivar(1,[4,1]);
Z2=lmivar(1,[4,1]);


%%Add the items 
lmiterm([1 1 1 P],1,A,'s');
lmiterm([1 1 1 Q1],1,1);
lmiterm([1 1 1 Q2],1,1);
lmiterm([1 1 1 Z1],-1,1);
lmiterm([1 1 2 Z1],1,1);
lmiterm([1 1 3 P],1,B*K);
lmiterm([1 1 5 Z1],d1*A',1);
lmiterm([1 1 6 Z2],d12*A',1);

lmiterm([1 2 2 Q1],-1,1);
lmiterm([1 2 2 Z1],-1,1);
lmiterm([1 2 2 Z2],-1,1);
lmiterm([1 2 3 Z2],1,1);

lmiterm([1 3 3 Z2],-2,1);
lmiterm([1 3 4 Z2],1,1);
lmiterm([1 3 5 Z1],d1*K'*B',1);
lmiterm([1 3 6 Z2],d12*K'*B',1);

lmiterm([1 4 4 Q2],-1,1);
lmiterm([1 4 4 Z2],-1,1);

lmiterm([1 5 5 Z1],-1,1);

lmiterm([1 6 6 Z2],-1,1);

lmiterm([2 1 1 P],-1,1);
lmiterm([3 1 1 Q1],-1,1);
lmiterm([4 1 1 Q2],-1,1);
lmiterm([5 1 1 Z1],-1,1);
lmiterm([6 1 1 Z2],-1,1);



%% Add the constraints of positive-definite matrix
lmiterm([2 1 1 P],-1,1);
lmiterm([3 1 1 Q1],-1,1);
lmiterm([4 1 1 Q2],-1,1);
lmiterm([5 1 1 Z1],-1,1);
lmiterm([6 1 1 Z2],-1,1);

%% Obtain the description of the LMI system
lmisys=getlmis;

%%Solve the LMI
[tmin,xfeas] = feasp(lmisys)
Pmat=dec2mat(lmisys,xfeas,P)
Q1mat=dec2mat(lmisys,xfeas,Q1)
Q2mat=dec2mat(lmisys,xfeas,Q2)
Z1mat=dec2mat(lmisys,xfeas,Z1)
Z2mat=dec2mat(lmisys,xfeas,Z2)







