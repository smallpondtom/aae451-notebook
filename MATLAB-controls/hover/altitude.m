%% HOVER ALTITUDE CONTROL 
% Hover is for 0 fps 
clear all; close all; clc;
% JSBsim plant
num = 62.71;  % numerator 
den = [1, 0, 0];  % denominator

Kp = 9.11459759968896e-10;
Kd = 0.00911454537591535;
N = 27.8343628363698;

c1 = N*Kd + Kp;
c2 = N*Kp;
c3 = 1;
c4 = N;