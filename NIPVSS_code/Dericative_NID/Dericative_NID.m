clear all;
clc;

%%Define the difference of two neighbouring network-induced delay
dott = ones(1,2535);

%%Define the sum of the difference of two neighbouring network-induced
%%delay in a long time inteval
sume = 0;

%%Load the data of network-induced delay and the image capturing time
%%instants
load .\Data_network_delay\ndy.mat;
load .\Data_triggering_time\trt.mat;
trt=trt';
trt=trt(2:2536);

%%Calculate the the sum of the difference of two neighbouring network-induced
%%delay in a long time inteval
for i=1:2534
   dott(i) = ndy(i+1)/((trt(i+1)-ndy(i+1))-(trt(i)-ndy(i)));
   sume = sume +dott(i);
end

%%The average of the the difference of two neighbouring network-induced
%%delay is set as the derivative of the network-induced delay
Derivative_NID = sume/2534;

