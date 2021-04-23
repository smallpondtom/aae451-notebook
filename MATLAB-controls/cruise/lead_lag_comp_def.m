Kp = 1.27231590533845e-09;
Kd = 0.0127229959138648;
N = 148.271702233295;

%% Lead-lag compensator def
C1 = N*Kd + Kp;
C2 = N*Kp;
C3 = 1;
C4 = N;

sys = sim('OL_cruise_pitch');
CL_sys = tf(conv([C1, C2], [1459]), conv([C3, C4], ;

figure(1)
rlocus(sys)