clear all;
clc;

%%Set the parameters of the inverted pendulum
m=0.109;%Mass of the pendulum
J=0.009083;%Moment of inertia of the pendulum
l=0.25;%Length from the pivot to the center of the pendulum
g=9.8;%Acceleration of gravity

%%Caculate and give the constant matrix in the state space.
A=[0 0 1 0;0 0 0 1;0 0 0 0;0 l*m*g/J 0 0]
B=[0;0;1;m*l/J]
C=[1 0 0 0;0 1 0 0]
D=[0;0]
Bw=[0.0015;0.007;0.075;0.35]

%%Set the controller
K=[3.7137 -29.3065 4.0140 -5.3880];


%% Set the delay
loop=1;
tao=0.014;
d1=0.019;
d2=0.072;
d3=d2+tao;
d12=d2-d1;
d13=d3-d1;
u1=0.3;   % The derivative of image-induced delay
u2=0.7;   % The derivative of network-induced delay

%% Set the disturbance attenuation level gamma
gama=2.1;

%% Identity matrix
I1=1;
I2=[1 0;0 1];


        %% Initialize LMI
        setlmis([]);
        %% Define the variables
        P=lmivar(1,[4,1]);
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

        %%Add the items 
        lmiterm([1 1 1 P],1,A,'s');
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
        lmiterm([1 1 7 P],1,B*K);
        lmiterm([1 1 9 P],1,Bw);
        lmiterm([1 1 10 0],C');
        lmiterm([1 1 11 Z1],d1*A',1);
        lmiterm([1 1 12 Z2],d12*A',1);
        lmiterm([1 1 13 Z3],tao*A',1);
        lmiterm([1 1 14 Z4],d13*A',1);


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
        lmiterm([1 7 10 0],K'*D');
        lmiterm([1 7 11 Z1],d1*K'*B',1);
        lmiterm([1 7 12 Z2],d12*K'*B',1);
        lmiterm([1 7 13 Z3],tao*K'*B',1);
        lmiterm([1 7 14 Z4],d13*K'*B',1);

        lmiterm([1 8 8 Q7],-1,1);
        lmiterm([1 8 8 Z4],-1,1);

        lmiterm([1 9 9 0],-gama*gama*I1);
        lmiterm([1 9 11 Z1],d1*Bw',1);
        lmiterm([1 9 12 Z2],d12*Bw',1);
        lmiterm([1 9 13 Z3],tao*Bw',1);
        lmiterm([1 9 14 Z4],d13*Bw',1);

        lmiterm([1 10 10 0],-I2);

        lmiterm([1 11 11 Z1],-1,1);
        lmiterm([1 12 12 Z2],-1,1);
        lmiterm([1 13 13 Z3],-1,1);
        lmiterm([1 14 14 Z4],-1,1);


        %% Add the constraints of positive-definite matrix
        lmiterm([2 1 1 P],-1,1);
        lmiterm([3 1 1 Q1],-1,1);
        lmiterm([4 1 1 Q2],-1,1);
        lmiterm([5 1 1 Q3],-1,1);
        lmiterm([6 1 1 Q4],-1,1);
        lmiterm([7 1 1 Q5],-1,1);
        lmiterm([8 1 1 Q6],-1,1);
        lmiterm([9 1 1 Q7],-1,1);
        lmiterm([10 1 1 Z1],-1,1);
        lmiterm([11 1 1 Z2],-1,1);
        lmiterm([12 1 1 Z3],-1,1);
        lmiterm([13 1 1 Z4],-1,1);

        %% Obtain the description of the LMI system
        lmisys=getlmis;
        
        %%Solve the LMI
        [tmin,xfeas] = feasp(lmisys)

        Pmat=dec2mat(lmisys,xfeas,P);
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







