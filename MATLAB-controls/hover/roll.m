%% HOVER ROLL CONTROL 
% Hover is for 0 fps 
clear all; close all; clc;
% JSBsim plant
num = 76.15;  % numerator 
den = [1, 0, 0];  % denominator

Kp = 7.66302615547072e-10;
Kd = 0.0076620813293718;
N = 505.040986830046;

c1 = N*Kd + Kp;
c2 = N*Kp;
c3 = 1;
c4 = N;