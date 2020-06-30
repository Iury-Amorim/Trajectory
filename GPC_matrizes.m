%Robot parameters
m3 = Mr(4); m2 = Mr(3); m1 = Mr(2); mrho = Mr(1);   
k3 = Kr(4); k2 = Kr(3); k1 = Kr(2); krho = Kr(1);
b3 = Br(4); b2 = Br(3); b1 = Br(2); brho = Br(1);

%% GPC design

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Tranfer matrix Model
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%delay
d = 0;
%disturbance model
c1aux = [1 0];
c2aux = [1 0];
c3aux = [1 0];
c4aux = [1 0];

A1 = [  0      1
    -k3/m3 -b3/m3];
B1 = [0; 1/m3];
C1 = [1 0];
D1 = 0;

A2 = [  0      1
    -k2/m2 -b2/m2];
B2 = [0; 1/m2];
C2 = [1 0];
D2 = 0;

A3 = [  0      1
    -k1/m1 -b1/m1];
B3 = [0; 1/m1];
C3 = [1 0];
D3 = 0;

A4 = [  0      1
    -krho/mrho -brho/mrho];
B4 = [0; 1/mrho];
C4 = [1 0];
D4 = 0;

[b1,a1] = ss2tf(A1,B1,C1,D1);
[b2,a2] = ss2tf(A2,B2,C2,D2);
[b3,a3] = ss2tf(A3,B3,C3,D3);
[b4,a4] = ss2tf(A4,B4,C4,D4);

y11c = tf(b1,a1);    y12c = 0;        y13c = 0;        y14c = 0;
y21c = 0;    y22c = tf(b2,a2);    y23c = 0;        y24c = 0;
y31c = 0;        y32c = 0;    y33c = tf(b3,a3);    y34c = 0;
y41c = 0;        y42c = 0;        y43c = 0;    y44c = tf(b4,a4);
s = tf('s');
##Plantac = [y11c y12c y13c y14c; y21c y22c y23c y24c; y31c y32c y33c y34c; y41c y42c y43c y44c].*exp(-d*s);
Plantac = [y11c y12c y13c y14c; y21c y22c y23c y24c; y31c y32c y33c y34c; y41c y42c y43c y44c];
[Ac,Bc,Cc,Dc] = ssdata(Plantac);

##ts = Ts*10;
ts = Ts;

%Discrete plant
y11d = c2d(y11c,ts);     y12d = 0;           y13d = 0;           y14d = 0;
y21d = 0;      y22d = c2d(y22c,ts);     y23d = 0;           y24d = 0;
y31d = 0;           y32d = 0;      y33d = c2d(y33c,ts);     y34d = 0;
y41d = 0;           y42d = 0;           y43d = 0;      y44d = c2d(y44c,ts);

Plantaz = [y11d y12d y13d y14d; y21d y22d y23d y24d; y31d y32d y33d y34d; y41d y42d y43d y44d];

[Az,Bz,Cz,Dz] = ssdata(Plantaz);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% CARIMA Model
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% delay operator
delta = tf([1 -1],1,ts,'Variable','z^-1');

% u1
[numy11d,deny11d] = tfdata(y11d,'v');
numy11d = tf(numy11d,1,ts,'Variable','z^-1');
deny11d = tf(deny11d,1,ts,'Variable','z^-1');
a11til = nonzeros(cell2mat(get(deny11d*delta,'num')));
b11til = nonzeros(cell2mat(get(numy11d*delta,'num')));
b11tilaux = [0;0;0];
c1 = conv(nonzeros(cell2mat(get(deny11d,'num')))',c1aux);

% u2
[numy22d,deny22d] = tfdata(y22d,'v');
numy22d = tf(numy22d,1,ts,'Variable','z^-1');
deny22d = tf(deny22d,1,ts,'Variable','z^-1');
a22til = nonzeros(cell2mat(get(deny22d*delta,'num')));
b22til = nonzeros(cell2mat(get(numy22d*delta,'num')));
b22tilaux = [0;0;0];
c2 = conv(c2aux,nonzeros(cell2mat(get(deny22d,'num')))');

% u3
[numy33d,deny33d] = tfdata(y33d,'v');
numy33d = tf(numy33d,1,ts,'Variable','z^-1');
deny33d = tf(deny33d,1,ts,'Variable','z^-1');
a33til = nonzeros(cell2mat(get(deny33d*delta,'num')));
b33til = nonzeros(cell2mat(get(numy33d*delta,'num')));
b33tilaux = [0;0;0];
c3 = conv(c3aux,nonzeros(cell2mat(get(deny33d,'num')))');

% u4
[numy44d,deny44d] = tfdata(y44d,'v');
numy44d = tf(numy44d,1,ts,'Variable','z^-1');
deny44d = tf(deny44d,1,ts,'Variable','z^-1');
a44til = nonzeros(cell2mat(get(deny44d*delta,'num')));
b44til = nonzeros(cell2mat(get(numy44d*delta,'num')));
b44tilaux = [0;0;0];
c4 = conv(c4aux,nonzeros(cell2mat(get(deny44d,'num')))');

%Augmented matrix
A1aux = [eye(length(a11til)-2); zeros(1,length(a11til)-2)];
A1 = [-a11til(2:end) A1aux];
B1 = [b11til b11tilaux b11tilaux b11tilaux];
H1 = [1 zeros(1,length(a11til)-2)];
D1 = c1'-a11til;
D1(1,:) = [];

A2aux = [eye(length(a22til)-2); zeros(1,length(a22til)-2)];
A2 = [-a22til(2:end) A2aux];
B2 = [b22tilaux b22til b22tilaux b22tilaux];
H2 = [1 zeros(1,length(a22til)-2)];
D2 = c2'-a22til;
D2(1,:) = [];

A3aux = [eye(length(a33til)-2); zeros(1,length(a33til)-2)];
A3 = [-a33til(2:end) A3aux];
B3 = [b33tilaux b33tilaux b33til b33tilaux];
H3 = [1 zeros(1,length(a33til)-2)];
D3 = c3'-a33til;
D3(1,:) = [];

A4aux = [eye(length(a44til)-2); zeros(1,length(a44til)-2)];
A4 = [-a44til(2:end) A4aux];
B4 = [b44tilaux b44tilaux b44tilaux b44til];
H4 = [1 zeros(1,length(a44til)-2)];
D4 = c4'-a44til;
D4(1,:) = [];

%matrizes aumentadas
[linA1, colA1] = size(A1);
[linD1, colD1] = size(D1);
[linH1, colH1] = size(H1);

Azeros = zeros(linA1,colA1);
Dzeros = zeros(linD1,colD1);
Hzeros = zeros(linH1,colH1);

Am = [A1 Azeros Azeros Azeros; Azeros A2 Azeros Azeros; Azeros Azeros A3 Azeros; Azeros Azeros Azeros A4];
Bm = [B1; B2; B3; B4];
Dm = [D1 Dzeros Dzeros Dzeros; Dzeros D2 Dzeros Dzeros; Dzeros Dzeros D3 Dzeros; Dzeros Dzeros Dzeros D4];
Hm = [H1 Hzeros Hzeros Hzeros; Hzeros H2 Hzeros Hzeros; Hzeros Hzeros H3 Hzeros; Hzeros Hzeros Hzeros H4];

[~, nIN] = size(Bm);

[linAz,colAz] = size(Az);
[linAm, colAm] = size(Am);
[linDm, colDm] = size(Dm);
[linHm, colHm] = size(Hm);

##Nu = 1; N = 1;
Nu = 5; N = 5;
[G,F,E,M,Q,R] = smatrixH(Nu,N,Am,Bm,Hm,Dm);
##%Q = Q*1e6; 
Q = Q*1e6; % Peso do Erro
##%R = R*1e-3;
R = R*1e-1; % Peso do Controle
##Q(1,1) = Q(1,1)*1e3;
##Q(2,2) = Q(2,2)*1e6;
##Q(3,3) = Q(3,3)*1e3;
##Q(4,4) = Q(4,4)*1e3;
