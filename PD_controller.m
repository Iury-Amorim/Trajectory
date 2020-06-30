syms z kv3 kp3 kv2 kp2 kv1 kp1 kvrho kprho
ts = 0.01
%% THETA 3
z1 = 0.95;
z2 = 0.9;

eq1 = -(-2-ts*kv3)/(1+ts*kv3+ts^2*kp3) == z1+z2;
eq2 = 1/(1+ts*kv3+ts^2*kp3) == z1*z2;

sol = solve(eq1,eq2,kv3,kp3);

kv3 = double(sol.kv3);
kp3 = double(sol.kp3);

T_3 = tf([0 0 1],[1 (-2-ts*kv3)/(1+ts*kv3+ts^2*kp3) 1/(1+ts*kv3+ts^2*kp3)],ts);

kv_th3_PD = kv3;
kp_th3_PD = kp3;

%% THETA 2
z1 = 0.8;
z2 = 0.8;

eq1 = -(-2-ts*kv2)/(1+ts*kv2+ts^2*kp2) == z1+z2;
eq2 = 1/(1+ts*kv2+ts^2*kp2) == z1*z2;

sol = solve(eq1,eq2,kv2,kp2);

kv2 = double(sol.kv2);
kp2 = double(sol.kp2);

T_2 = tf([0 0 1],[1 (-2-ts*kv2)/(1+ts*kv2+ts^2*kp2) 1/(1+ts*kv2+ts^2*kp2)],ts);

kv_th2_PD = kv2;
kp_th2_PD = kp2;

%% THETA 1
z1 = 0.8;
z2 = 0.8;

eq1 = -(-2-ts*kv1)/(1+ts*kv1+ts^2*kp1) == z1+z2;
eq2 = 1/(1+ts*kv1+ts^2*kp1) == z1*z2;

sol = solve(eq1,eq2,kv1,kp1);

kv1 = double(sol.kv1);
kp1 = double(sol.kp1);

T_1 = tf([0 0 1],[1 (-2-ts*kv1)/(1+ts*kv1+ts^2*kp1) 1/(1+ts*kv1+ts^2*kp1)],ts);

kv_th1_PD = kv1;
kp_th1_PD = kp1;

%% RHO
z1 = 0.8;
z2 = 0.8;

eq1 = -(-2-ts*kvrho)/(1+ts*kvrho+ts^2*kprho) == z1+z2;
eq2 = 1/(1+ts*kvrho+ts^2*kprho) == z1*z2;

sol = solve(eq1,eq2,kvrho,kprho);

kvrho = double(sol.kvrho);
kprho = double(sol.kprho);

T_rho = tf([0 0 1],[1 (-2-ts*kvrho)/(1+ts*kvrho+ts^2*kprho) 1/(1+ts*kvrho+ts^2*kprho)],ts);

kv_rho_PD = kvrho;
kp_rho_PD = kprho;