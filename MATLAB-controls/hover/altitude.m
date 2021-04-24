%% HOVER ALTITUDE CONTROL 
% Hover is for 0 fps 
clear all; close all; clc;
% JSBsim plant
num = 62.71;  % numerator 
den = [1, 0, 0];  % denominator

Kp = 0.0004;
Kd = 0.0045;
N = 0.6;

c1 = N*Kd + Kp;
c2 = N*Kp;
c3 = 1;
c4 = N;

s = tf('s');
G = tf(num, den);
H = (c1*s + c2) / (c3*s + c4);
bw = bandwidth(feedback(G*H, 1));