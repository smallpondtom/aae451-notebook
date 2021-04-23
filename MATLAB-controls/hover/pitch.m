%% HOVER PITCH CONTROL 
% Hover is for 0 fps 

% JSBsim plant
num = 41.6;  % numerator 
den = [1, 0, 0];  % denominator

Kp = 1.27231590533845e-09;
Kd = 0.0127229959138648;
N = 148.271702233295;

c1 = N*Kd + Kp;
c2 = N*Kp;
c3 = 1;
c4 = N;