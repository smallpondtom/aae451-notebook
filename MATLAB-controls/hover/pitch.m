%% HOVER PITCH CONTROL 
% Hover is for 0 fps 

% JSBsim plant
num = 41.6;  % numerator 
den = [1, 0, 0];  % denominator

Kp = 0.005;
Kd = 0.3;
N = 15;

c1 = N*Kd + Kp;
c2 = N*Kp;
c3 = 1;
c4 = N;

s = tf('s');
G = tf(num, den);
H = (c1*s + c2) / (c3*s + c4);
bw = bandwidth(feedback(G*H, 1));