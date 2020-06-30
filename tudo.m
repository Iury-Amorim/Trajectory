%SIMULACAO DE UM MANIPULADOR DE 3LINK

clc, clear -v, close all
pkg load symbolic
pkg load control
setenv PYTHON python

%% GERADOR DE TRAJETORIAS

Ts = 0.01;
operator = 5;

switch operator
    case 1 %% trajetoria circular 
        
    th = pi/2:Ts:5*pi/2;
    X =3 * cos(th); % deixar em 3
    Z =3 * sin(th); % deixar em 3
    z_c = (20.5+Z); % deixar 26/27
    x_c = (0+X);        
    y_c = 2.1 +zeros(1,length(x_c)); % deixa 1.4
    
    z_reta = [z_c(1)+3:-0.01:z_c(1)];
    x_reta = zeros(1,length(z_reta));
    y_reta = y_c(1)*ones(1,length(z_reta));
    
    x = [x_reta x_c];
    z_ = [z_reta z_c];
    y = [y_reta y_c];

    case 2 %% Trajetoria espiral 

    a = 0.7;
    theta = 0:Ts:(9*pi/2);
    r_pos= a*(theta.^(1/2));

    for i=1:length(r_pos)
    X(i) = (r_pos(i)*cos(theta(i)));
    Z(i) = (r_pos(i)*sin(theta(i)));
    end
    X = fliplr(X);
    Z = fliplr(Z);
    x1 = [X(1):Ts:X(1)+2];
    z1 = Z(1)*ones(1,length(x1));
    z_e = 20+Z;
    x_e =  0+X;
    y_e = 2.1 +zeros(1,length(x_e)); % deixa 1.4
    
    z_reta = [z_e(1)+3:-0.01:z_e(1)];
    x_reta = zeros(1,length(z_reta));
    y_reta = y_e(1)*ones(1,length(z_reta));    
    
    x = [x_reta x_e];
    z_ = [z_reta z_e];
    y = [y_reta y_e];
    
%    plot3(x,z_,y)
%    break
    case 3 % trejatoria senoidal
        
    a = 3
    X = ([0:Ts:4*pi]);
    Z = a.*sin(X) ;

    x_s = -5+[X];
    z_s = 24+Z;    
    y_s = 2.1 +zeros(1,length(x_s)); % deixa 1.4
    
    
    x_reta = [x_s(1)-3:+0.01:x_s(1)];
    z_reta = z_s(1)+ zeros(1,length(x_reta));
    y_reta = y_s(1)*ones(1,length(x_reta));    
    
    x = [x_reta x_s];
    z_ = [z_reta z_s];
    y = [y_reta y_s];
    
    
%     plot3(x,z_,y)
%     break

    case 4 %trajetoria triangular
    p_ini = 8;
    dist = 3;
    z1 = [p_ini:-Ts:dist];
    x1 = [0:dist/(length(z1)-1):dist];
    x2 = x1(end):-Ts:-dist;
    z2 = z1(end)*ones(1,length(x2));
    z3 =[dist:+Ts:p_ini];
    x3 = [x2(end):dist/(length(z1)-1):0];
    z_t = 15+[z1 z2 z3];
    x_t = 0+[x1 x2 x3];
    y_t = 2.1 +zeros(1,length(x_t)); % deixa 1.4
    
    z_reta = [z_t(1)+2:-0.01:z_t(1)];
    x_reta = x_t(1) + zeros(1,length(z_reta));
    y_reta = y_t(1)*ones(1,length(z_reta));
    
    
    x = [x_reta x_t];
    z_ = [z_reta z_t];
    y = [y_reta y_t];
    
    case 5 %circuito boost
    % Loading trajectory data
    load("pontos.mat")
    track = 'Circuit'
    X = []; Y = []; Z = [];
    xPD = []; yPD = []; zPD = []; xPDf = []; yPDf = []; zPDf = [];
    xGPC = []; yGPC = []; zGPC = []; xGPCf = []; yGPCf = []; zGPCf = [];

    x = []; y =[]; z_ = [];
    for sp = 1:size(Track,2)
    %% Trajectory generator
        x = [x; Track{sp}(:,1)];
        y = [y; Track{sp}(:,2)];
        z_ = [z_; Track{sp}(:,3)];
    end

    X = x(1:end-1); Y = y(1:end-1); Z = z_(1:end-1);

    x = -4+ X;
    z_ = 18+Y;
    y = Z;
    
%     plot3(x,z_,y)
%     break
end





% -------cinematica inversa 3-links

  l1=12.8;
  l2=12.8;
  l3 = 8;

for i=1:length(x)
    r(i) = sqrt(x(i)^2 + z_(i)^2);
    phi(i) = -atan2(y(i),r(i));
    beta(i) = atan2(y(i)-l3*sin(phi(i)),r(i) - l3*cos(phi(i)));
    qsi(i) = acos(((r(i) - l3*cos(phi(i)))^2 + (y(i) - l3*sin(phi(i)))^2 + l1^2 - l2^2)/(2*l1*sqrt((r(i) - l3*cos((phi(i))))^2 + (y(i) - l3*sin(phi(i)))^2)));
    
    rho_pd(i) = atan2(z_(i),x(i))-pi/2;
    theta1_pd(i) = (beta(i) + qsi(i))-pi/2;
    theta2_pd(i) = -acos(((r(i) - l3*cos(phi(i)))^2 + (y(i) - l3*sin(phi(i)))^2 - l1^2 - l2^2)/(2*l1*l2));
    theta3_pd(i) = -(phi(i) - theta1_pd(i)-pi/2- theta2_pd(i));
end




pos_ini = [rho_pd(1) -theta1_pd(1) -theta2_pd(1) -theta3_pd(1)];

angles = [rho_pd' -theta1_pd' -theta2_pd' -theta3_pd'];

sim =remApiSetup();
simxFinish(-1); % stop the current program that is running
clientID=simxStart('127.0.0.1',19999,true,true,50,5);
%simxStart(clientID,sim.simx_opmode_oneshot_wait)
if (clientID ~= -1)
    disp('conectado');
    
        % --------------------- Setup the simulation
   
    simxSynchronous(clientID,true)
    
    joint_names_char = ['PhantomXPincher_joint1 ' 'PhantomXPincher_joint2 ' 'PhantomXPincher_joint3 ' 'PhantomXPincher_joint4'];
    joint_names = strsplit(joint_names_char);
    
    % get the handles for each joint and set up streaming
    joint_handles = zeros(1, length(joint_names));
    for name=1:length(joint_names)
        [~,joint_handles(1,name)] = simxGetObjectHandle(clientID, joint_names{name}, sim.simx_opmode_blocking);
    end

    % get handle for target and set up streaming
    dt = 0.01;
    simxSetFloatingParameter(clientID,sim.sim_floatparam_simulation_time_step,dt,sim.simx_opmode_oneshot);
        
        % --------------------- Start the simulation

    % start our simulation in lockstep with our code
    simxStartSimulation(clientID,sim.simx_opmode_blocking);
    q = zeros(1,length(joint_handles));
    dq = zeros(1,length(joint_handles));
    
    q_ini = zeros(1,length(joint_handles));
    dq_ini = zeros(1,length(joint_handles));
    
    for i=1:length(joint_names)
        simxSetJointTargetPosition(clientID,joint_handles(i),pos_ini(i),sim.simx_opmode_oneshot); %Movimento junta 4    
        simxSetJointTargetVelocity(clientID,joint_handles(i),1.74533,sim.simx_opmode_blocking); % target velocity
        pause(0.01);
        [returnCode, q_ini(i)] = simxGetJointPosition(clientID,joint_handles(i),sim.simx_opmode_blocking) ;
    end  
    pause(2) ;
    
    
%    break   
    count = 0;
    track_xyz_pd = [];
    q_ant = [0 0 0 0];
    dq_ant = [0 0 0 0];
    angles_ant = [0 0 0 0];
    Q_pd = [0 0 0 0];
    DQ = [0 0 0 0];
    U_pd = [];
    i = 1;
    PD_controller;
        M = [0.294 0.239 0.144 0.072]; % mass in order Link1 to Link4
        K = [15.221650282442914  4.102535681804911  8.906406088691533   7.615900695306100];
        B = [2.197541637788649   2.167639031769078   1.626567906732705  1.365414107094439];
        N = [15.022770655158023   4.675739192460690    9.023690264188282   7.847587370572287];
         tam = 0.01;
    while i <= length(rho_pd) % run for 1 simulated second

        
        for ii=1:length(joint_handles)
            % get the joint angles
            [returnCode, q(ii)] = simxGetJointPosition(clientID,joint_handles(ii),sim.simx_opmode_blocking);
            [returnCode, dq(ii)] = simxGetObjectFloatParameter(clientID,joint_handles(ii),2012, sim.simx_opmode_blocking); % parameter ID for angular velocity of the joint
        end
        Q_pd = [Q_pd ; q];
        DQ = [DQ; dq];

%        %------- CONTROL LAW -----------------------
        % ELO 1
        
        erro1 = angles(i,1) - q(1);
        erro1_derivada = (angles(i,1) - angles_ant(1))/tam - (dq(1));
        ref1_derivada = (dq(1));

        u1 =  (M(1)*(kp_rho_PD*erro1 + kv_rho_PD*erro1_derivada) + (B(1)*ref1_derivada + K(1)*q(1)));
        % ELO 2
        erro2 = angles(i,2)-q(2);
        erro2_derivada = (angles(i,2) - angles_ant(2))/tam - (dq(2));
        ref2_derivada = (dq(2));
   
        u2 =  (M(2)*(kp_th1_PD*erro2 + kv_th1_PD*erro2_derivada) + (B(2)*ref2_derivada + K(2)*q(2)));
        % ELO 3
        erro3 = angles(i,3)-q(3);
        erro3_derivada = (angles(i,3) - angles_ant(3))/tam - (dq(3));
        ref3_derivada = (dq(3));
   
        u3 =  (M(3)*(kp_th2_PD*erro3 + kv_th2_PD*erro3_derivada) + (B(3)*ref3_derivada + K(3)*q(3)));
        % ELO 4
        erro4 = angles(i,4)-q(4);
        erro4_derivada = (angles(i,4) - angles_ant(4))/tam - (dq(4));
        ref4_derivada = (dq(4));
   
        u4 =  (M(4)*(kp_th3_PD*erro4 + kv_th3_PD*erro4_derivada) + (B(4)*ref4_derivada + K(4)*q(4)));
         
          
        u = [u1/N(1) u2/N(2) u3/N(3) u4/N(4)];
        
       
        U_pd = [U_pd; u]; 
        q_ant = q;
        angles_ant = angles(i,:);
         
         
        for ii=1:length(joint_handles)
            [returnCode, torque]=simxGetJointForce(clientID,joint_handles(ii),sim.simx_opmode_blocking);
              
            simxSetJointTargetPosition(clientID,joint_handles(ii),u(ii),sim.simx_opmode_oneshot); %Movimento junta 4    
            simxSetJointTargetVelocity(clientID,joint_handles(ii),0.0174533,sim.simx_opmode_blocking); % target velocity

        end
        
        % move simulation ahead one time step
        simxSynchronousTrigger(clientID);
        count = count + 1  ;
        i = i+1;

    end      
    
else
    disp('Failed connecting to remote API server');
end


% stop the simulation
%simxStopSimulation(clientID, sim.simx_opmode_blocking);

% Before closing the connection to V-REP,
% make sure that the last command sent out had time to arrive.
simxGetPingTime(clientID);

% Now close the connection to V-REP:
%simxFinish(clientID)
%disp('connection closed...')

%% Plot figure

%Plot error angles
Matix_erro1_pd = (rho_pd')    -Q_pd(2:end,1);
Matix_erro2_pd = (-theta1_pd')-Q_pd(2:end,2);
Matix_erro3_pd = (-theta2_pd')-Q_pd(2:end,3);
Matix_erro4_pd = (-theta3_pd')-Q_pd(2:end,4);

global track1 track2 track3 track4

cont = 0;
for i=1:length(x)
    track1 = Q_pd(i,1); %rho
    track2 = -Q_pd(i,2); %theta1
    track3 = -Q_pd(i,3); %theta2
    track4 = -Q_pd(i,4); %theta3
    
    fun = @getTrajectory;

    tr0 = [x(i),y(i),z_(i)];
    
    [tr,info] = fsolve(fun,tr0);
    
    if info == -3
      teste = 1;
     end
    
##if tr(1)>2*x(i) || tr(2)>2*y(i) || tr(3)>2*z_(i)
  if tr(2)>2*y(i) || tr(3)>2*z_(i)
        tr1(i) = x(i);
        tr2(i) = y(i);
        tr3(i) = z_(i);
        cont = cont+1;
    else
        tr1(i) = tr(1);
        tr2(i) = tr(2);
        tr3(i) = tr(3);
    end
end

%% Ajuste para artigo

tr1_pd = tr1;
tr2_pd = tr2;
tr3_pd = tr3;

 pause(20)

%SIMULACAO DE UM MANIPULADOR DE 3LINK

clc, clear -x operator Q_pd tr1_pd tr2_pd tr3_pd Matix_erro1_pd Matix_erro2_pd Matix_erro3_pd Matix_erro4_pd U_pd, close all

pkg load symbolic
pkg load control
setenv PYTHON python


%% GERADOR DE TRAJETORIAS

Ts = 0.01;
%operator = 2;

switch operator
    case 1 %% trajetoria circular 
        
    th = pi/2:Ts:5*pi/2;
    X =3 * cos(th); % deixar em 3
    Z =3 * sin(th); % deixar em 3
    z_c = (20.5+Z); % deixar 26/27
    x_c = (0+X);        
    y_c = 2.1 +zeros(1,length(x_c)); % deixa 1.4
    
    z_reta = [z_c(1)+3:-0.01:z_c(1)];
    x_reta = zeros(1,length(z_reta));
    y_reta = y_c(1)*ones(1,length(z_reta));
    
    x = [x_reta x_c];
    z_ = [z_reta z_c];
    y = [y_reta y_c];

    case 2 %% Trajetoria espiral 
    a = 0.7;
    theta = 0:Ts:(9*pi/2);
    r_pos= a*(theta.^(1/2));

    for i=1:length(r_pos)
    X(i) = (r_pos(i)*cos(theta(i)));
    Z(i) = (r_pos(i)*sin(theta(i)));
    end
    X = fliplr(X);
    Z = fliplr(Z);
    x1 = [X(1):Ts:X(1)+2];
    z1 = Z(1)*ones(1,length(x1));
    z_e = 20+Z;
    x_e =  0+X;
    y_e = 2.1 +zeros(1,length(x_e)); % deixa 1.4
    
    z_reta = [z_e(1)+3:-0.01:z_e(1)];
    x_reta = zeros(1,length(z_reta));
    y_reta = y_e(1)*ones(1,length(z_reta));    
    
    x = [x_reta x_e];
    z_ = [z_reta z_e];
    y = [y_reta y_e];
    
%    plot3(x,z_,y)
%    break
    
    case 3 % trejatoria senoidal
        
    a = 3
    X = ([0:Ts:4*pi]);
    Z = a.*sin(X) ;

    x_s = -5+[X];
    z_s = 24+Z;    
    y_s = 2.1 +zeros(1,length(x_s)); % deixa 1.4
    
    
    x_reta = [x_s(1)-3:+0.01:x_s(1)];
    z_reta = z_s(1)+ zeros(1,length(x_reta));
    y_reta = y_s(1)*ones(1,length(x_reta));    
    
    x = [x_reta x_s];
    z_ = [z_reta z_s];
    y = [y_reta y_s];
    
    
%     plot3(x,z_,y)
%     break
    case 4 %trajetoria triangular
    p_ini = 8;
    dist = 3;
    z1 = [p_ini:-Ts:dist];
    x1 = [0:dist/(length(z1)-1):dist];
    x2 = x1(end):-Ts:-dist;
    z2 = z1(end)*ones(1,length(x2));
    z3 =[dist:+Ts:p_ini];
    x3 = [x2(end):dist/(length(z1)-1):0];
    z_t = 15+[z1 z2 z3];
    x_t = 0+[x1 x2 x3];
    y_t = 2.1 +zeros(1,length(x_t)); % deixa 1.4
    
    z_reta = [z_t(1)+2:-0.01:z_t(1)];
    x_reta = x_t(1) + zeros(1,length(z_reta));
    y_reta = y_t(1)*ones(1,length(z_reta));
    
    
    x = [x_reta x_t];
    z_ = [z_reta z_t];
    y = [y_reta y_t];
    
    
%     plot3(x,z_,y)
%     break

    case 5 %circuito boost
    % Loading trajectory data
    load("pontos.mat")
    track = 'Circuit'
    X = []; Y = []; Z = [];
    xPD = []; yPD = []; zPD = []; xPDf = []; yPDf = []; zPDf = [];
    xGPC = []; yGPC = []; zGPC = []; xGPCf = []; yGPCf = []; zGPCf = [];

    x = []; y =[]; z_ = [];
    for sp = 1:size(Track,2)
    %% Trajectory generator
        x = [x; Track{sp}(:,1)];
        y = [y; Track{sp}(:,2)];
        z_ = [z_; Track{sp}(:,3)];
    end

    X = x(1:end-1); Y = y(1:end-1); Z = z_(1:end-1);

    x = -4+ X;
    z_ = 18+Y;
    y = Z;
end





% -------cinematica inversa 3-links

l1=12.8;
l2=12.8;
l3 = 8;



for i=1:length(x)
    r(i) = sqrt(x(i)^2 + z_(i)^2);
    phi(i) = -atan2(y(i),r(i));
    beta(i) = atan2(y(i)-l3*sin(phi(i)),r(i) - l3*cos(phi(i)));
    qsi(i) = acos(((r(i) - l3*cos(phi(i)))^2 + (y(i) - l3*sin(phi(i)))^2 + l1^2 - l2^2)/(2*l1*sqrt((r(i) - l3*cos((phi(i))))^2 + (y(i) - l3*sin(phi(i)))^2)));
    
    rho(i) = atan2(z_(i),x(i))-pi/2;
    theta1(i) = (beta(i) + qsi(i))-pi/2;
    theta2(i) = -acos(((r(i) - l3*cos(phi(i)))^2 + (y(i) - l3*sin(phi(i)))^2 - l1^2 - l2^2)/(2*l1*l2));
    theta3(i) = -(phi(i) - theta1(i)-pi/2- theta2(i));
end

pos_ini = [rho(1) -theta1(1) -theta2(1) -theta3(1)];

angles = [rho' -theta1' -theta2' -theta3'];


sim =remApiSetup();
simxFinish(-1); % stop the current program that is running
clientID=simxStart('127.0.0.1',19999,true,true,50,5);
if (clientID ~= -1)
    disp('conectado');
    
        % --------------------- Setup the simulation
   
    simxSynchronous(clientID,true)
    
    joint_names_char = ['PhantomXPincher_joint1 ' 'PhantomXPincher_joint2 ' 'PhantomXPincher_joint3 ' 'PhantomXPincher_joint4'];
    joint_names = strsplit(joint_names_char);
    
    % joint target velocities discussed below
##    joint_target_velocities = ones(1,length(joint_names))*10000.0
    
    % get the handles for each joint and set up streaming
    joint_handles = zeros(1, length(joint_names));
    for name=1:length(joint_names)
        [~,joint_handles(1,name)] = simxGetObjectHandle(clientID, joint_names{name}, sim.simx_opmode_blocking);
    end

    % get handle for target and set up streaming
    dt = 0.01;
    simxSetFloatingParameter(clientID,sim.sim_floatparam_simulation_time_step,dt,sim.simx_opmode_oneshot);
        
        % --------------------- Start the simulation

    % start our simulation in lockstep with our code
    simxStartSimulation(clientID,sim.simx_opmode_blocking);
    q = zeros(1,length(joint_handles));
    dq = zeros(1,length(joint_handles));
    
    q_ini = zeros(1,length(joint_handles));
    dq_ini = zeros(1,length(joint_handles));
    
    for i=1:length(joint_names)
        simxSetJointTargetPosition(clientID,joint_handles(i),pos_ini(i),sim.simx_opmode_oneshot); %Movimento junta 4    
        simxSetJointTargetVelocity(clientID,joint_handles(i),1.74533,sim.simx_opmode_blocking); % target velocity
        pause(0.01);
        [returnCode, q_ini(i)] = simxGetJointPosition(clientID,joint_handles(i),sim.simx_opmode_blocking) ;
    end  
    pause(2) ;
    
    
    
%    break   
    count = 0;
    track_xyz_pd = [];
    q_ant = [0 0 0 0];
    dq_ant = [0 0 0 0];
    angles_ant = [0 0 0 0];
    erro1f_ant = [0 0 0 0];
    erro2f_ant = [0 0 0 0];
    erro3f_ant = [0 0 0 0];
    erro4f_ant = [0 0 0 0];
    Q = [0 0 0 0];
    DQ = [0 0 0 0];
    U_f = [];
    U_pd_ = [];
    U_pdf = [];
    i = 1;
    PD_controller;
    M = [0.294 0.239 0.144 0.072]; % mass in order Link1 to Link4
        K = [15.221650282442914  4.102535681804911  8.906406088691533   7.615900695306100];
        B = [2.197541637788649   2.167639031769078   1.626567906732705  1.365414107094439];
        N = [15.022770655158023   4.675739192460690    9.023690264188282   7.847587370572287];
 
        tam = 0.01;
        tam2 = 1;
        gamath3PD = 100;
        gamath2PD = 100;
        gamath1PD = 100;
        gamarhoPD = 100;
        
  gama = [gamarhoPD gamath1PD gamath2PD gamath3PD];
    while i <= length(rho) % run for 1 simulated second
     
        for ii=1:length(joint_handles)
            % get the joint angles
            [returnCode, q(ii)] = simxGetJointPosition(clientID,joint_handles(ii),sim.simx_opmode_blocking);

            % get the joint velocity
            [returnCode, dq(ii)] = simxGetObjectFloatParameter(clientID,joint_handles(ii),2012, sim.simx_opmode_blocking); % parameter ID for angular velocity of the joint

        end
        Q = [Q ; q];
        DQ = [DQ; dq];
        %        L = [l1  l2]*10^(-2); % arm segment lengths

%        %------- CONTROL LAW -----------------------
%        q = q;
%        dq = dq;

        % ELO 1        
        erro1 = angles(i,1) - q(1);
        erro1_derivada = (angles(i,1) - angles_ant(1))/tam - (dq(1));
        erro1f_derivada =  (erro1 - erro1f_ant)/tam2;   
        ref1_derivada = (dq(1));
        
         %Fuzzy_1
         Kc_1 = CtrlFuzzy(erro1,erro1f_derivada);         
         %PD_1
         u1 =  (M(1)*(kp_rho_PD*erro1 + kv_rho_PD*erro1_derivada) + (B(1)*ref1_derivada + K(1)*q(1)));

        
        % ELO 2
        erro2 = angles(i,2)-q(2);
        erro2_derivada = (angles(i,2) - angles_ant(2))/tam - (dq(2));
        erro2f_derivada =  (erro2 - erro2f_ant)/tam2;   
        ref2_derivada = (dq(2));
      
       %Fuzzy_2
        Kc_2 = CtrlFuzzy(erro2,erro2f_derivada) ;     
        %PD_2   
        u2 =  (M(2)*(kp_th1_PD*erro2 + kv_th1_PD*erro2_derivada) + (B(2)*ref2_derivada + K(2)*q(2)));
        
        
        % ELO 3
        erro3 = angles(i,3)-q(3);
        erro3_derivada = (angles(i,3) - angles_ant(3))/tam - (dq(3));
        erro3f_derivada =  (erro3 - erro3f_ant)/tam2; 
        ref3_derivada = (dq(3));
        
       %Fuzzy_3
        Kc_3 = CtrlFuzzy(erro3,erro3f_derivada)   ; 
        u3 =  (M(3)*(kp_th2_PD*erro3 + kv_th2_PD*erro3_derivada) + (B(3)*ref3_derivada + K(3)*q(3)));


        % ELO 4
        erro4 = angles(i,4)-q(4);
        erro4_derivada = (angles(i,4) - angles_ant(4))/tam - (dq(4));
        erro4f_derivada =  (erro4 - erro4f_ant)/tam2;
        ref4_derivada = (dq(4));

       %Fuzzy_4
        Kc_4 = CtrlFuzzy(erro4,erro4f_derivada) ;
        u4 =  (M(4)*(kp_th3_PD*erro4 + kv_th3_PD*erro4_derivada) + (B(4)*ref4_derivada + K(4)*q(4)));
         
          
        u_pd = [u1/N(1) u2/N(2) u3/N(3) u4/N(4)];
        u_f = [(Kc_1*gama(1))/N(1) (Kc_2*gama(2))/N(2) (Kc_3*gama(3))/N(3) (Kc_4*gama(4))/N(4)];
##        u_pd = [u1 u2 u3 u4];
##        u_f = [(Kc_1*gama(1)) (Kc_2*gama(2)) (Kc_3*gama(3)) (Kc_4*gama(4))];
        
        u = [(u_pd(1)+u_f(1)) (u_pd(2)+u_f(2)) (u_pd(3)+u_f(3)) (u_pd(4)+u_f(4))];
        
        
        
        
%        U_pd = [U_pd; u_pd]

        U_pdf = [U_pdf;u]; 
        U_pd_ = [U_pd_;u_pd];
        U_f = [U_f;u_f];
        q_ant = q;
        erro1f_ant= erro1;
        erro2f_ant= erro2;
        erro3f_ant= erro3;
        erro4f_ant= erro4;
        angles_ant = angles(i,:);
         
         
        for ii=1:length(joint_handles)
 
            % get the current joint torque
            [returnCode, torque]=simxGetJointForce(clientID,joint_handles(ii),sim.simx_opmode_blocking);
            
            % if force has changed signs,
            % we need to change the target velocity sign
%             if sign(torque)*sign(u(ii)) < 0
%                 joint_target_velocities(ii) = joint_target_velocities(ii)*-1
%                 vrep.simxSetJointTargetVelocity(clientID,joint_handles(ii),joint_target_velocities(ii),vrep.simx_opmode_blocking) % target velocity
%             end
            
%            simxSetJointTargetPosition(clientID,joint_handles(ii),angles(i,ii),sim.simx_opmode_oneshot); %Movimento junta 4    
            simxSetJointTargetPosition(clientID,joint_handles(ii),u(ii),sim.simx_opmode_oneshot); %Movimento junta 4    
            simxSetJointTargetVelocity(clientID,joint_handles(ii),0.0174533,sim.simx_opmode_blocking); % target velocity

            %            if returnCode ~=0 
%                disp('Failed connecting to remote API server')
%                break                   
%            end
        end
        
        % move simulation ahead one time step
        simxSynchronousTrigger(clientID);
        count = count + 1  ;
        i = i+1;
        

         

    end
    
    % stop the simulation
%    simxStopSimulation(clientID, sim.simx_opmode_blocking);

    % Before closing the connection to V-REP,
    % make sure that the last command sent out had time to arrive.
%    simxGetPingTime(clientID);

    % Now close the connection to V-REP:
%    simxFinish(clientID) ;   
        
    
else
    disp('Failed connecting to remote API server')
end


% stop the simulation
%simxStopSimulation(clientID, sim.simx_opmode_blocking);

% Before closing the connection to V-REP,
% make sure that the last command sent out had time to arrive.
simxGetPingTime(clientID);

% Now close the connection to V-REP:
%simxFinish(clientID);
%disp('connection closed...')

%% Plot figure
Matix_erro1_pdf = (rho')    -Q(2:end,1);
Matix_erro2_pdf = (-theta1')-Q(2:end,2);
Matix_erro3_pdf = (-theta2')-Q(2:end,3);
Matix_erro4_pdf = (-theta3')-Q(2:end,4);

%% trajectory
global track1 track2 track3 track4
cont_F = 0;
for i=1:length(x)
    track1 = Q(i,1); %rho
    track2 = -Q(i,2); %theta1
    track3 = -Q(i,3); %theta2
    track4 = -Q(i,4); %theta3
    
    fun_F = @getTrajectory;

    tr0_F = [x(i),y(i),z_(i)];
    
    [tr_F,info] = fsolve(fun_F,tr0_F);
    
##if tr_F(1)>2*x(i) || tr_F(2)>2*y(i) || tr_F(3)>2*z_(i)
  if tr_F(2)>2*y(i) || tr_F(3)>2*z_(i)
        tr1_F(i) = x(i);
        tr2_F(i) = y(i);
        tr3_F(i) = z_(i);
        cont_F = cont_F+1;
    else
        tr1_F(i) = tr_F(1);
        tr2_F(i) = tr_F(2);
        tr3_F(i) = tr_F(3);
    end
end

%% Ajuste para artigo

tr1_pd_fuzzy = tr1_F;
tr2_pd_fuzzy = tr2_F;
tr3_pd_fuzzy = tr3_F;


pause(20)

%SIMULACAO DE UM MANIPULADOR DE 3LINK
clc, clear -x operator tr1_pd tr2_pd tr3_pd tr1_pd_fuzzy tr2_pd_fuzzy tr3_pd_fuzzy ...
U_pd U_pdf Q_pd Q, close all
setenv PYTHON python
pkg load symbolic
pkg load control
pkg load signal

%% GERADOR DE TRAJETORIAS

Ts = 0.01;
%operator = 2;

switch operator
    case 1 %% trajetoria circular 
        
    th = pi/2:Ts:5*pi/2;
    X =3 * cos(th); % deixar em 3
    Z =3 * sin(th); % deixar em 3
    z_c = (20.5+Z); % deixar 26/27
    x_c = (0+X);        
    y_c = 2.1 +zeros(1,length(x_c)); % deixa 1.4
    
    z_reta = [z_c(1)+3:-0.01:z_c(1)];
    x_reta = zeros(1,length(z_reta));
    y_reta = y_c(1)*ones(1,length(z_reta));
    
    x = [x_reta x_c];
    z_ = [z_reta z_c];
    y = [y_reta y_c];

    case 2 %% Trajetoria espiral 

    a = 0.7;
    theta = 0:Ts:(9*pi/2);
    r_pos= a*(theta.^(1/2));

    for i=1:length(r_pos)
    X(i) = (r_pos(i)*cos(theta(i)));
    Z(i) = (r_pos(i)*sin(theta(i)));
    end
    X = fliplr(X);
    Z = fliplr(Z);
    x1 = [X(1):Ts:X(1)+2];
    z1 = Z(1)*ones(1,length(x1));
    z_e = 20+Z;
    x_e =  0+X;
    y_e = 2.1 +zeros(1,length(x_e)); % deixa 1.4
    
    z_reta = [z_e(1)+3:-0.01:z_e(1)];
    x_reta = zeros(1,length(z_reta));
    y_reta = y_e(1)*ones(1,length(z_reta));    
    
    x = [x_reta x_e];
    z_ = [z_reta z_e];
    y = [y_reta y_e];
    
%    plot3(x,z_,y)
%    break
    case 3 % trejatoria senoidal
        
    a = 3
    X = ([0:Ts:4*pi]);
    Z = a.*sin(X) ;

    x_s = -5+[X];
    z_s = 24+Z;    
    y_s = 2.1 +zeros(1,length(x_s)); % deixa 1.4
    
    
    x_reta = [x_s(1)-3:+0.01:x_s(1)];
    z_reta = z_s(1)+ zeros(1,length(x_reta));
    y_reta = y_s(1)*ones(1,length(x_reta));    
    
    x = [x_reta x_s];
    z_ = [z_reta z_s];
    y = [y_reta y_s];
    
    
%     plot3(x,z_,y)
%     break

    case 4 %trajetoria triangular
    p_ini = 8;
    dist = 3;
    z1 = [p_ini:-Ts:dist];
    x1 = [0:dist/(length(z1)-1):dist];
    x2 = x1(end):-Ts:-dist;
    z2 = z1(end)*ones(1,length(x2));
    z3 =[dist:+Ts:p_ini];
    x3 = [x2(end):dist/(length(z1)-1):0];
    z_t = 15+[z1 z2 z3];
    x_t = 0+[x1 x2 x3];
    y_t = 2.1 +zeros(1,length(x_t)); % deixa 1.4
    
    z_reta = [z_t(1)+2:-0.01:z_t(1)];
    x_reta = x_t(1) + zeros(1,length(z_reta));
    y_reta = y_t(1)*ones(1,length(z_reta));
    
    
    x = [x_reta x_t];
    z_ = [z_reta z_t];
    y = [y_reta y_t];
    
    
%     plot3(x,z_,y)
%     break

    case 5 %circuito boost
    % Loading trajectory data
    load("pontos.mat")
    track = 'Circuit'
    X = []; Y = []; Z = [];
    xPD = []; yPD = []; zPD = []; xPDf = []; yPDf = []; zPDf = [];
    xGPC = []; yGPC = []; zGPC = []; xGPCf = []; yGPCf = []; zGPCf = [];

    x = []; y =[]; z_ = [];
    for sp = 1:size(Track,2)
    %% Trajectory generator
        x = [x; Track{sp}(:,1)];
        y = [y; Track{sp}(:,2)];
        z_ = [z_; Track{sp}(:,3)];
    end

    X = x(1:end-1); Y = y(1:end-1); Z = z_(1:end-1);

    x = -4+ X;
    z_ = 18+Y;
    y = Z;
    
end
  
  % -------cinematica inversa 3-links
  
  l1=12.8;
  l2=12.8;
  l3 = 8;
  
  for i=1:length(x)
    r(i) = sqrt(x(i)^2 + z_(i)^2);
    phi(i) = -atan2(y(i),r(i));
    beta(i) = atan2(y(i)-l3*sin(phi(i)),r(i) - l3*cos(phi(i)));
    qsi(i) = acos(((r(i) - l3*cos(phi(i)))^2 + (y(i) - l3*sin(phi(i)))^2 + l1^2 - l2^2)/(2*l1*sqrt((r(i) - l3*cos((phi(i))))^2 + (y(i) - l3*sin(phi(i)))^2)));
    
    rho(i) = atan2(x(i),z_(i));
    theta1(i) = (beta(i) + qsi(i))-pi/2;
    theta2(i) = -acos(((r(i) - l3*cos(phi(i)))^2 + (y(i) - l3*sin(phi(i)))^2 - l1^2 - l2^2)/(2*l1*l2));
    theta3(i) = -(phi(i) - theta1(i)-pi/2- theta2(i));
  end
  
  
  
  
  pos_ini = [rho(1) -theta1(1) -theta2(1) -theta3(1)];
  
  angles = [rho' -theta1' -theta2' -theta3']; % Referencia
  
  % Projeto do Controlador
  Mr = [0.294 0.239 0.144 0.072]; % mass in order Link1 to Link4
  Kr = [15.221650282442914  4.102535681804911  8.906406088691533   7.615900695306100];
  Br = [2.197541637788649   2.167639031769078   1.626567906732705  1.365414107094439];
  Nr = [15.022770655158023   4.675739192460690    9.023690264188282   7.847587370572287];
  GPC_matrizes;
  
%Ganhos
K1_ = pinv(G'*Q*G + M'*R*M)*G'*Q;
K2_ = pinv(G'*Q*G + M'*R*M)*M'*R;

order = nIN;
K1 = [];
K2 = [];
for i = 1:order
    K1 = [K1; K1_(i,:)];
    K2 = [K2; K2_(i,1:order)];
end

KF = K1*F;
KE = K1*E;
KW = zeros(4,4);
for i = 1:N
    KW = KW+K1(:,4*i-3:4*i);
end
  %Referencia
  W  = zeros(linHm,1); 
  auxw = 1:linHm;
  %re = [-theta3;-theta2;-theta1;rho];
  re = [-theta3;-theta2;-theta1;rho];
  %condi��es iniciais
  e_est = zeros(nIN,1);
  uo  = zeros(nIN,1);
  xf = zeros(linAm,1);
  
  sinal_matrix_dbg = zeros(nIN,1);

  %    break   
    count = 0;
    track_xyz = [];
    q_ant = [0 0 0 0];
    dq_ant = [0 0 0 0];
    angles_ant = [0 0 0 0];
    erro1f_ant = [0 0 0 0];
    erro2f_ant = [0 0 0 0];
    erro3f_ant = [0 0 0 0];
    erro4f_ant = [0 0 0 0];
    Qc = [0 0 0 0];
    DQ = [0 0 0 0];
    U = [];
    i = 1;
  Yplanta = [0 0 0 0]';
  % Iniciar a comunication com o Coppelia 
  sim =remApiSetup();
  simxFinish(-1); % stop the current program that is running
  clientID=simxStart('127.0.0.1',19999,true,true,50,5);
  if (clientID ~= -1)
    disp('conectado');
    % --------------------- Setup the simulation
    simxSynchronous(clientID,true);
    joint_names_char = ['PhantomXPincher_joint1 ' 'PhantomXPincher_joint2 ' 'PhantomXPincher_joint3 ' 'PhantomXPincher_joint4'];
    joint_names = strsplit(joint_names_char);
    % get the handles for each joint and set up streaming
    joint_handles = zeros(1, length(joint_names));
    for name=1:length(joint_names)
      [~,joint_handles(1,name)] = simxGetObjectHandle(clientID, joint_names{name}, sim.simx_opmode_blocking);
    end
    % get handle for target and set up streaming
    dt = 0.01;
    simxSetFloatingParameter(clientID,sim.sim_floatparam_simulation_time_step,dt,sim.simx_opmode_oneshot);
    
    % --------------------- Start the simulation --------------------------------------------------------------%
    % start our simulation in lockstep with our code
    simxStartSimulation(clientID,sim.simx_opmode_blocking);
    q = zeros(1,length(joint_handles));   % Posi��es
    dq = zeros(1,length(joint_handles));  % Velocidades
    
    q_ini = zeros(1,length(joint_handles));
    dq_ini = zeros(1,length(joint_handles));
    
    for ii=1:length(joint_names)
      simxSetJointTargetPosition(clientID,joint_handles(ii),pos_ini(ii),sim.simx_opmode_oneshot); %Movimento junta 4    
      simxSetJointTargetVelocity(clientID,joint_handles(ii),1.74533,sim.simx_opmode_blocking); % target velocity
      pause(0.01)
      [returnCode, q_ini(ii)] = simxGetJointPosition(clientID,joint_handles(ii),sim.simx_opmode_blocking) ;
    end  
    pause(2)
     
    i = 1;
    while i <= length(rho) % run for 1 simulated second
      %origin of system
      [returnCode,origin] = simxGetObjectHandle(clientID,'PhantomXPincher_joint1', sim.simx_opmode_blocking);
      [returnCode,target_handle] = simxGetObjectHandle(clientID,'feltPen_tip', sim.simx_opmode_blocking);
      % get the angular position of the reference trajectory 
      [~,xyz_real] = simxGetObjectPosition(clientID,target_handle,origin,sim.simx_opmode_blocking); % retrieve absolute, not relative, position
      track_xyz = [track_xyz  xyz_real];
      
      for ii=1:length(joint_handles)
        % get the joint angles
        [returnCode, q(ii)] = simxGetJointPosition(clientID,joint_handles(ii),sim.simx_opmode_blocking);
        % get the joint velocity
        [returnCode, dq(ii)] = simxGetObjectFloatParameter(clientID,joint_handles(ii),2012, sim.simx_opmode_blocking); % parameter ID for angular velocity of the joint
       end 
       % joint 1 = (rho), ..., joint4(theta3)
       Qc = [Qc ; q];
##       DQ = [DQ; dq];
       auxYplanta = [ q(4) q(3) q(2) q(1)]';
       Yplanta = [Yplanta auxYplanta];
%%%%---------------------------------------------- Controle do GPC -------------------------------------------------------%%%%        
    if i <= length(rho)-N
        %Considerando a��o preditiva
        for k = 1:N
            aux = auxw+(k-1)*linHm;
            W(aux) = re(:,i+Nu);
        end
    else
        for k = 1:N
            aux = auxw+(k-1)*linHm;
            W(aux) = re(:,i);
        end
    end
     e_est(:,i) = Yplanta(:,i)-Hm*xf(:,i);
    
    
    u(:,i) = KW*re(:,i) - (KF*xf(:,i) + KE*e_est(:,i)) + K2*uo;
    u(1,i) = u(1,i)/Nr(4);
    u(2,i) = u(2,i)/Nr(3);
    u(3,i) = u(3,i)/Nr(2);
    u(4,i) = u(4,i)/Nr(1);
    
%     u(:,i) = 5*(re(:,i)-Yplanta(:,i));
    
    f = F*xf(:,i) + E*e_est(:,i);
##    Uo = [uo; zeros((Nu-1)*nIN,1)];
##    U = K1_*(W-f) + K2_*Uo;
##    obj(i) = U'*(G'*(Q)*G+M'*(R)*M)*U + 2*(G'*(Q')*(f-W)-M'*(R')*Uo)'*U; % aptid�
    
    uo = u(:,i);
    
    % predi��o
    xf(:,i+1) = Am*xf(:,i) + Bm*u(:,i) + Dm*e_est(:,i);
  
   %---------Debug init ------%
     clc
     saida_dbg = auxYplanta;
     ref_dbg = re(:,i);
     residuo_dbg = e_est(:,i); ;
     sinal_controle_dbg = u(:,i);
     sinal_matrix_dbg = [sinal_matrix_dbg u(:,i)];
     erro_dbg = re(:,i) - auxYplanta;
    %---------Debug fim ------%
% Transformando a sa�da do controlador para a planta
    ux(1) = u(4,i);%rho
    ux(2) = u(3,i);%theta1
    ux(3) = u(2,i);%theta2
    ux(4) = u(1,i);%theta3
    
        for ii=1:length(joint_handles)
          % get the current joint torque
          [returnCode, torque]=simxGetJointForce(clientID,joint_handles(ii),sim.simx_opmode_blocking);
        
          simxSetJointTargetPosition(clientID,joint_handles(ii),ux(ii),sim.simx_opmode_oneshot); %Movimento junta 4    
          simxSetJointTargetVelocity(clientID,joint_handles(ii),0.0174533,sim.simx_opmode_blocking); % target velocity
        end
        
        % move simulation ahead one time step
        simxSynchronousTrigger(clientID);
        count = count + 1;  
        i = i+1;
        
      end
      
      % stop the simulation
%      simxStopSimulation(clientID, sim.simx_opmode_blocking);
      
      % Before closing the connection to V-REP,
      % make sure that the last command sent out had time to arrive.
%      simxGetPingTime(clientID);
      
      % Now close the connection to V-REP:
%      simxFinish(clientID)   ; 
      
      
    else
      disp('Failed connecting to remote API server')
    end    
    
    
    % stop the simulation
%    simxStopSimulation(clientID, sim.simx_opmode_blocking);
    simxGetPingTime(clientID);
% Plot do Erro de cada junta
Matix_erro1 = (rho')    -Qc(2:end,1);
Matix_erro2 = (-theta1')-Qc(2:end,2);
Matix_erro3 = (-theta2')-Qc(2:end,3);
Matix_erro4 = (-theta3')-Qc(2:end,4);
    
 %teste de cinematica
global track1 track2 track3 track4

cont = 0;
for i=1:length(x)
    track1 = Qc(i,1); %rho
    track2 = -Qc(i,2); %theta1
    track3 = -Qc(i,3); %theta2
    track4 = -Qc(i,4); %theta3
    
    fun = @getTrajectory;

    tr0 = [x(i),y(i),z_(i)];
    
    [tr,info] = fsolve(fun,tr0);
    
    if info == -3
      teste = 1;
     end
   
  if tr(2)>2*y(i) || tr(3)>2*z_(i)
        tr1(i) = x(i);
        tr2(i) = y(i);
        tr3(i) = z_(i);
        cont = cont+1;
    else
        tr1(i) = tr(1);
        tr2(i) = tr(2);
        tr3(i) = tr(3);
    end
end

%figure (4)
%plot3(x,-z_,y,'b');
%hold on
%plot3(-tr1,-tr3,tr2,'r')
%legend("Referencia");
%title("Trajetoria de referencia");


%% Ajuste para artigo
tr1_gpc = tr1;
tr2_gpc = tr2;
tr3_gpc = tr3;

pause(20)




%SIMULACAO DE UM MANIPULADOR DE 3LINK
clc, clear -x operator tr1_pd tr2_pd tr3_pd tr1_pd_fuzzy tr2_pd_fuzzy tr3_pd_fuzzy ...
U_pd U_pdf Q_pd Q Qc tr1_gpc tr2_gpc tr3_gpc Matix_erro1 Matix_erro2 Matix_erro3 Matix_erro4, close all
setenv PYTHON python
pkg load symbolic
pkg load control
pkg load signal

%% GERADOR DE TRAJETORIAS

Ts = 0.01;
%operator = 2;

switch operator
    case 1 %% trajetoria circular 
    track = 'Circular'   
    th = pi/2:Ts:5*pi/2;
    X =3 * cos(th); % deixar em 3
    Z =3 * sin(th); % deixar em 3
    z_c = (20.5+Z); % deixar 26/27
    x_c = (0+X);        
    y_c = 2.1 +zeros(1,length(x_c)); % deixa 1.4
    
    z_reta = [z_c(1)+3:-0.01:z_c(1)];
    x_reta = zeros(1,length(z_reta));
    y_reta = y_c(1)*ones(1,length(z_reta));
    
    x = [x_reta x_c];
    z_ = [z_reta z_c];
    y = [y_reta y_c];

    case 2 %% Trajetoria espiral 
    track = 'Spiral'
    a = 0.7;
    theta = 0:Ts:(9*pi/2);
    r_pos= a*(theta.^(1/2));

    for i=1:length(r_pos)
    X(i) = (r_pos(i)*cos(theta(i)));
    Z(i) = (r_pos(i)*sin(theta(i)));
    end
    X = fliplr(X);
    Z = fliplr(Z);
    x1 = [X(1):Ts:X(1)+2];
    z1 = Z(1)*ones(1,length(x1));
    z_e = 20+Z;
    x_e =  0+X;
    y_e = 2.1 +zeros(1,length(x_e)); % deixa 1.4
    
    z_reta = [z_e(1)+3:-0.01:z_e(1)];
    x_reta = zeros(1,length(z_reta));
    y_reta = y_e(1)*ones(1,length(z_reta));    
    
    x = [x_reta x_e];
    z_ = [z_reta z_e];
    y = [y_reta y_e];
    
%    plot3(x,z_,y)
%    break
    case 3 % trejatoria senoidal
    track = 'Sinusoidal'   
    a = 3
    X = ([0:Ts:4*pi]);
    Z = a.*sin(X) ;

    x_s = -5+[X];
    z_s = 24+Z;    
    y_s = 2.1 +zeros(1,length(x_s)); % deixa 1.4
    
    
    x_reta = [x_s(1)-3:+0.01:x_s(1)];
    z_reta = z_s(1)+ zeros(1,length(x_reta));
    y_reta = y_s(1)*ones(1,length(x_reta));    
    
    x = [x_reta x_s];
    z_ = [z_reta z_s];
    y = [y_reta y_s];
    
    
%     plot3(x,z_,y)
%     break

    case 4 %trajetoria triangular
    track = 'Triangular'  
      
    p_ini = 8;
    dist = 3;
    z1 = [p_ini:-Ts:dist];
    x1 = [0:dist/(length(z1)-1):dist];
    x2 = x1(end):-Ts:-dist;
    z2 = z1(end)*ones(1,length(x2));
    z3 =[dist:+Ts:p_ini];
    x3 = [x2(end):dist/(length(z1)-1):0];
    z_t = 15+[z1 z2 z3];
    x_t = 0+[x1 x2 x3];
    y_t = 2.1 +zeros(1,length(x_t)); % deixa 1.4
    
    z_reta = [z_t(1)+2:-0.01:z_t(1)];
    x_reta = x_t(1) + zeros(1,length(z_reta));
    y_reta = y_t(1)*ones(1,length(z_reta));
    
    
    x = [x_reta x_t];
    z_ = [z_reta z_t];
    y = [y_reta y_t];
    
    
%     plot3(x,z_,y)
%     break

    case 5 %circuito boost
    % Loading trajectory data
    load("pontos.mat")
    track = 'Circuit'
    X = []; Y = []; Z = [];
    xPD = []; yPD = []; zPD = []; xPDf = []; yPDf = []; zPDf = [];
    xGPC = []; yGPC = []; zGPC = []; xGPCf = []; yGPCf = []; zGPCf = [];

    x = []; y =[]; z_ = [];
    for sp = 1:size(Track,2)
    %% Trajectory generator
        x = [x; Track{sp}(:,1)];
        y = [y; Track{sp}(:,2)];
        z_ = [z_; Track{sp}(:,3)];
    end

    X = x(1:end-1); Y = y(1:end-1); Z = z_(1:end-1);

    x = -4+ X;
    z_ = 18+Y;
    y = Z;


%    plot3(x,z_,y)         
%    break       
      
end
  
  % -------cinematica inversa 3-links
  
  l1=12.8;
  l2=12.8;
  l3 = 8;
  
  for i=1:length(x)
    r(i) = sqrt(x(i)^2 + z_(i)^2);
    phi(i) = -atan2(y(i),r(i));
    beta(i) = atan2(y(i)-l3*sin(phi(i)),r(i) - l3*cos(phi(i)));
    qsi(i) = acos(((r(i) - l3*cos(phi(i)))^2 + (y(i) - l3*sin(phi(i)))^2 + l1^2 - l2^2)/(2*l1*sqrt((r(i) - l3*cos((phi(i))))^2 + (y(i) - l3*sin(phi(i)))^2)));
    
    rho(i) = atan2(x(i),z_(i));
    theta1(i) = (beta(i) + qsi(i))-pi/2;
    theta2(i) = -acos(((r(i) - l3*cos(phi(i)))^2 + (y(i) - l3*sin(phi(i)))^2 - l1^2 - l2^2)/(2*l1*l2));
    theta3(i) = -(phi(i) - theta1(i)-pi/2- theta2(i));
  end
  
  
  
  
  pos_ini = [rho(1) -theta1(1) -theta2(1) -theta3(1)];
  
  angles = [rho' -theta1' -theta2' -theta3']; % Referencia
  
  % Projeto do Controlador
  Mr = [0.294 0.239 0.144 0.072]; % mass in order Link1 to Link4
  Kr = [15.221650282442914  4.102535681804911  8.906406088691533   7.615900695306100];
  Br = [2.197541637788649   2.167639031769078   1.626567906732705  1.365414107094439];
  Nr = [15.022770655158023   4.675739192460690    9.023690264188282   7.847587370572287];
  GPC_matrizes;
  %Altera��es para o FUzzy
##  R = 10*R; %g
  
%Ganhos
K1_ = pinv(G'*Q*G + M'*R*M)*G'*Q;
K2_ = pinv(G'*Q*G + M'*R*M)*M'*R;

order = nIN;
K1 = [];
K2 = [];
for i = 1:order
    K1 = [K1; K1_(i,:)];
    K2 = [K2; K2_(i,1:order)];
end

KF = K1*F;
KE = K1*E;
KW = zeros(4,4);
for i = 1:N
    KW = KW+K1(:,4*i-3:4*i);
end
  %Referencia
  W  = zeros(linHm,1); 
  auxw = 1:linHm;
  %re = [-theta3;-theta2;-theta1;rho];
  re = [-theta3;-theta2;-theta1;rho];
  %condi��es iniciais
  e_est = zeros(nIN,1);
  uo  = zeros(nIN,1);
  xf = zeros(linAm,1);
  % vARIAVEIS PRA PLOT
  CTRL_FUZZ = zeros(1,nIN);
  CTRL_GPC = zeros(1,nIN);
  CTRL_GPC_FUZZ = zeros(1,nIN);
 
  %    break   
    count = 0;
    track_xyz = [];
    q_ant = [0 0 0 0];
    dq_ant = [0 0 0 0];
    angles_ant = [0 0 0 0];
    erro1f_ant = [0 0 0 0];
    erro2f_ant = [0 0 0 0];
    erro3f_ant = [0 0 0 0];
    erro4f_ant = [0 0 0 0];
    Qc_f = [0 0 0 0];
    DQ = [0 0 0 0];
    U = [];
    i = 1;
    U_final = [];
  Yplanta = [0 0 0 0]';
  % Iniciar a comunication com o Coppelia 
  sim =remApiSetup();
  simxFinish(-1); % stop the current program that is running
  clientID=simxStart('127.0.0.1',19999,true,true,50,5);
  if (clientID ~= -1)
    disp('conectado');
    % --------------------- Setup the simulation
    simxSynchronous(clientID,true);
    joint_names_char = ['PhantomXPincher_joint1 ' 'PhantomXPincher_joint2 ' 'PhantomXPincher_joint3 ' 'PhantomXPincher_joint4'];
    joint_names = strsplit(joint_names_char);
    % get the handles for each joint and set up streaming
    joint_handles = zeros(1, length(joint_names));
    for name=1:length(joint_names)
      [~,joint_handles(1,name)] = simxGetObjectHandle(clientID, joint_names{name}, sim.simx_opmode_blocking);
    end
    % get handle for target and set up streaming
    dt = 0.01;
    simxSetFloatingParameter(clientID,sim.sim_floatparam_simulation_time_step,dt,sim.simx_opmode_oneshot);
    
    % --------------------- Start the simulation --------------------------------------------------------------%
    % start our simulation in lockstep with our code
    simxStartSimulation(clientID,sim.simx_opmode_blocking);
    q = zeros(1,length(joint_handles));   % Posi��es
    dq = zeros(1,length(joint_handles));  % Velocidades
    
    q_ini = zeros(1,length(joint_handles));
    dq_ini = zeros(1,length(joint_handles));
    
    for ii=1:length(joint_names)
      simxSetJointTargetPosition(clientID,joint_handles(ii),pos_ini(ii),sim.simx_opmode_oneshot); %Movimento junta 4    
      simxSetJointTargetVelocity(clientID,joint_handles(ii),1.74533,sim.simx_opmode_blocking); % target velocity
      pause(0.01)
      [returnCode, q_ini(ii)] = simxGetJointPosition(clientID,joint_handles(ii),sim.simx_opmode_blocking) ;
    end  
    pause(2) 
i = 1;    

          tam = 1;%0.01;
          gamath3GPC = 100; % g200 - 150 - 150
          gamath2GPC = 100; % g100 - 100 - 150
          gamath1GPC = 100; % g300 - 200 - 150
          gamarhoGPC = 100; % g600 - 200 - 150
          
          gama = [gamarhoGPC gamath1GPC gamath2GPC gamath3GPC];
    while i <= length(rho) % run for 1 simulated second
      %origin of system
      [returnCode,origin] = simxGetObjectHandle(clientID,'PhantomXPincher_joint1', sim.simx_opmode_blocking);
      [returnCode,target_handle] = simxGetObjectHandle(clientID,'feltPen_tip', sim.simx_opmode_blocking);
      % get the angular position of the reference trajectory 
      [~,xyz_real] = simxGetObjectPosition(clientID,target_handle,origin,sim.simx_opmode_blocking); % retrieve absolute, not relative, position
      track_xyz = [track_xyz  xyz_real];
      
      for ii=1:length(joint_handles)
        % get the joint angles
        [returnCode, q(ii)] = simxGetJointPosition(clientID,joint_handles(ii),sim.simx_opmode_blocking);
        % get the joint velocity
        [returnCode, dq(ii)] = simxGetObjectFloatParameter(clientID,joint_handles(ii),2012, sim.simx_opmode_blocking); % parameter ID for angular velocity of the joint
       end 
       % joint 1 = (rho), ..., joint4(theta3)
       Qc_f = [Qc_f ; q];
       DQ = [DQ; dq];
       auxYplanta = [ q(4) q(3) q(2) q(1)]';
       Yplanta = [Yplanta auxYplanta];
%%%%---------------------------------------------- Controle do GPC -------------------------------------------------------%%%%        
    if i <= length(rho)-N
        %Considerando a��o preditiva
        for k = 1:N
            aux = auxw+(k-1)*linHm;
            W(aux) = re(:,i+Nu);
        end
    else
        for k = 1:N
            aux = auxw+(k-1)*linHm;
            W(aux) = re(:,i);
        end
    end
     e_est(:,i) = Yplanta(:,i)-Hm*xf(:,i);
    
    
    u(:,i) = KW*re(:,i) - (KF*xf(:,i) + KE*e_est(:,i)) + K2*uo;
    u(1,i) = u(1,i)/Nr(4);
    u(2,i) = u(2,i)/Nr(3);
    u(3,i) = u(3,i)/Nr(2);
    u(4,i) = u(4,i)/Nr(1);
    
%     u(:,i) = 5*(re(:,i)-Yplanta(:,i));
    
    f = F*xf(:,i) + E*e_est(:,i);
##    Uo = [uo; zeros((Nu-1)*nIN,1)];
##    U = K1_*(W-f) + K2_*Uo;
##    obj(i) = U'*(G'*(Q)*G+M'*(R)*M)*U + 2*(G'*(Q')*(f-W)-M'*(R')*Uo)'*U; % aptid�
    
    uo = u(:,i);
    
    % predi��o
    xf(:,i+1) = Am*xf(:,i) + Bm*u(:,i) + Dm*e_est(:,i);
  
  
% Transformando a sa�da do controlador para a planta
    ux(1) = u(4,i);%rho
    ux(2) = u(3,i);%theta1
    ux(3) = u(2,i);%theta2
    ux(4) = u(1,i);%theta3
    
% ----------------------- Controle Fuzzy ------------------- %
          
          %        q = q;
          %        dq = dq;
          
          % ELO 1        
          erro1 = angles(i,1) - q(1);
##          erro1_derivada = (angles(i,1) - angles_ant(1))/tam - (dq(1));
          erro1f_derivada =  (erro1 - erro1f_ant);   
##          ref1_derivada = (dq(1));
          
          %Fuzzy_1
          Kc_1 = CtrlFuzzy(erro1,erro1f_derivada);                  
          
          % ELO 2
          erro2 = angles(i,2)-q(2);
##          erro2_derivada = (angles(i,2) - angles_ant(2))/tam - (dq(2));
          erro2f_derivada =  (erro2 - erro2f_ant);   
##          ref2_derivada = (dq(2));
          
          %Fuzzy_2
          Kc_2 = CtrlFuzzy(erro2,erro2f_derivada) ;     
           
          
          % ELO 3
          erro3 = angles(i,3)-q(3);
##          erro3_derivada = (angles(i,3) - angles_ant(3))/tam - (dq(3));
          erro3f_derivada =  (erro3 - erro3f_ant); 
##          ref3_derivada = (dq(3));
          
          %Fuzzy_3
          Kc_3 = CtrlFuzzy(erro3,erro3f_derivada)    ;  
          
          % ELO 4
          erro4 = angles(i,4)-q(4);
##          erro4_derivada = (angles(i,4) - angles_ant(4))/tam - (dq(4));
          erro4f_derivada =  (erro4 - erro4f_ant);
##          ref4_derivada = (dq(4));
          
          %Fuzzy_4
          Kc_4 = CtrlFuzzy(erro4,erro4f_derivada);     
          
          u_gpc = [ux(1) ux(2) ux(3) ux(4)];
          u_fuzzy = [(Kc_1*gama(1))/Nr(1) (Kc_2*gama(2))/Nr(2)  (Kc_3*gama(3))/Nr(3)  (Kc_4*gama(4))/Nr(4)];
          u_gpc_fuzzy = u_gpc + u_fuzzy; 
          
          U_final = [U_final ;u_gpc_fuzzy]; 
          q_ant = q;
          erro1f_ant= erro1;
          erro2f_ant= erro2;
          erro3f_ant= erro3;
          erro4f_ant= erro4;
          angles_ant = angles(i,:);
  
% ------- Control Fuzzy END ---%
    %---------Debug init ------%
     clc
     i;
     saida_dbg = auxYplanta;
     ref_dbg = re(:,i);
     sinal_ctrl_gpc_dbg = u_gpc';
     sinal_ctrl_fuzz_dbg = u_fuzzy';
     sinal_ctrl_gpc_fuzzy_dgb = u_gpc_fuzzy';
     CTRL_FUZZ = [CTRL_FUZZ; u_fuzzy]; % Usado pra plotar o Sinal de Controle FUZZY
     CTRL_GPC_FUZZ = [CTRL_GPC_FUZZ; u_gpc_fuzzy];
     CTRL_GPC = [CTRL_GPC; u_gpc]; % Usado pra plotar o Sinal de Controle GPC
     erro_dbg = re(:,i) - auxYplanta;
    %---------Debug fim ------%
    
    
        for ii=1:length(joint_handles)
          % get the current joint torque
          [returnCode, torque]=simxGetJointForce(clientID,joint_handles(ii),sim.simx_opmode_blocking);
        
          simxSetJointTargetPosition(clientID,joint_handles(ii),u_gpc_fuzzy(ii),sim.simx_opmode_oneshot); %Movimento junta 4    
          simxSetJointTargetVelocity(clientID,joint_handles(ii),0.0174533,sim.simx_opmode_blocking); % target velocity
        end
        
        % move simulation ahead one time step
        simxSynchronousTrigger(clientID);
        count = count + 1;  
        i = i+1;
        
      end
      
      % stop the simulation
      simxStopSimulation(clientID, sim.simx_opmode_blocking);
      
      % Before closing the connection to V-REP,
      % make sure that the last command sent out had time to arrive.
      simxGetPingTime(clientID);
      
      % Now close the connection to V-REP:
      simxFinish(clientID)   ; 
      
      
    else
      disp('Failed connecting to remote API server')
    end    
    
    
    % stop the simulation
    simxStopSimulation(clientID, sim.simx_opmode_blocking);
    simxGetPingTime(clientID);
    simxFinish(clientID);
    disp('connection closed...')
    
% Plot dos �ngulos de cada junta
%dados_GPC_plotar;
%    h = figure (1);
%    subplot(4,1,1)
%    plot(rho','b')
%    hold on
%    plot(Qc_f(2:end,1),'r')
%    hold on
%    plot(Qc(2:end,1),'g')
%    legend("reference","GPC-FUZZY","GPC", "location", "northeastoutside");
%    set (gca, "ygrid", "on");
%    title("Rho-angulos")
%    
%    subplot(4,1,2)
%    plot(-theta1','b')
%    hold on
%    plot(Qc_f(2:end,2),'r')
%    hold on
%    plot(Qc(2:end,2),'g')
%    legend("reference","GPC-FUZZY","GPC", "location", "northeastoutside");
%    set (gca, "ygrid", "on");
%    title("Theta1-angulos")
%    
%    
%    subplot(4,1,3)
%    plot(-theta2','b')
%    hold on
%    plot(Qc_f(2:end,3),'r')
%    hold on
%    plot(Qc(2:end,3),'g')
%    legend("reference","GPC-FUZZY","GPC", "location", "northeastoutside");
%    set (gca, "ygrid", "on");
%    title("Theta2-angulos")
%    
%    
%    subplot(4,1,4)
%    plot(-theta3','b')
%    hold on
%    plot(Qc_f(2:end,4),'r')
%    hold on
%    plot(Qc(2:end,4),'g')
%    legend("reference","GPC-FUZZY","GPC", "location", "northeastoutside");
%    set (gca, "ygrid", "on");
%    title("Theta3-angulos")
    
%    movegui(h,'north')
##    saveas (h, photo_gpc_fuzzy_angle, jpg);
% Plot dos Sinais de Controle de cada junta
%   h = figure(2);
%    subplot(4,1,1)
%    plot(CTRL_FUZZ(10:end,1),'r')
%    hold on
%    plot(CTRL_GPC(10:end,1),'g')
%    legend("CTRL-GPC-FUZZY","GPC", "location", "northeastoutside");
%    set (gca, "ygrid", "on");
%    title("Rho-sinal controle")
%    
%    subplot(4,1,2)
%    plot(CTRL_FUZZ(10:end,2),'r')
%    hold on
%    plot(CTRL_GPC(10:end,2),'g')
%    legend("CTRL-GPC-FUZZY","GPC", "location", "northeastoutside");
%    set (gca, "ygrid", "on");
%    title("Theta1-sinal controle")
%    
%    subplot(4,1,3)
%     plot(CTRL_FUZZ(10:end,3),'r')
%    hold on
%    plot(CTRL_GPC(10:end,3),'g')
%    legend("CTRL-GPC-FUZZY","GPC", "location", "northeastoutside");
%    set (gca, "ygrid", "on");
%    title("Theta2-sinal controle")
%    
%    subplot(4,1,4)
%    plot(CTRL_FUZZ(10:end,4),'r')
%    hold on
%    plot(CTRL_GPC(10:end,4),'g')
%    legend("CTRL-GPC-FUZZY","GPC", "location", "northeastoutside");
%    set (gca, "ygrid", "on");
%    title("Theta3-sinal controle")
    
%    movegui(h,'southwest')
##saveas (h, photo_gpc_fuzzy_control, jpg);
% Plot do Erro de cada junta
GPC_erro_plotar;
Erro_gpc_fuzzy1 = (rho')    -Qc_f(2:end,1);
Erro_gpc_fuzzy2 = (-theta1')-Qc_f(2:end,2);
Erro_gpc_fuzzy3 = (-theta2')-Qc_f(2:end,3);
Erro_gpc_fuzzy4 = (-theta3')-Qc_f(2:end,4);

%h =  figure(3);
%    subplot(4,1,1)
%    plot(Erro_gpc_fuzzy1,'r')
%    hold on
%    plot(Matix_erro1,'g')  
%    title("Rho-erro")
%    set (gca, "ygrid", "on");
%    legend("GPC-FUZZY","GPC", "location", "northeastoutside");
%    
%    subplot(4,1,2)
%    plot(Erro_gpc_fuzzy2,'r')
%    hold on
%    plot(Matix_erro2,'g')
%    legend("GPC-FUZZY","GPC", "location", "northeastoutside");
%    set (gca, "ygrid", "on");
%    title("Theta1-erro")
%     
%    subplot(4,1,3)
%    plot(Erro_gpc_fuzzy3,'r')
%    hold on
%    plot(Matix_erro3,'g')
%    legend("GPC-FUZZY","GPC", "location", "northeastoutside");
%    set (gca, "ygrid", "on");
%    title("Theta2-erro")
%     
%    subplot(4,1,4)
%    plot(Erro_gpc_fuzzy4,'r')
%    hold on
%    plot(Matix_erro1,'g')
%    legend("GPC-FUZZY","GPC", "location", "northeastoutside");
%    set (gca, "ygrid", "on");
%    title("Theta3-erro")
     
%    movegui(h,'south')
##    saveas (h, photo_gpc_fuzzy_control, jpg);
    
 %teste de cinematica
global track1 track2 track3 track4

cont = 0;
for i=1:length(x)
    track1 = Qc_f(i,1); %rho
    track2 = -Qc_f(i,2); %theta1
    track3 = -Qc_f(i,3); %theta2
    track4 = -Qc_f(i,4); %theta3
    
    fun = @getTrajectory;

    tr0 = [x(i),y(i),z_(i)];
    
    [tr,info] = fsolve(fun,tr0);
    
    if info == -3
      teste = 1;
     end
   
  if tr(2)>2*y(i) || tr(3)>2*z_(i)
        tr1_f(i) = x(i);
        tr2_f(i) = y(i);
        tr3_f(i) = z_(i);
        cont = cont+1;
    else
        tr1_f(i) = tr(1);
        tr2_f(i) = tr(2);
        tr3_f(i) = tr(3);
    end
end
cont
%figure (4)
%plot3(x,z_,y,'b');
%hold on
%plot3(-tr1_gpc,tr3_gpc,tr2_gpc,'r')
%hold on
%plot3(-tr1_f,tr3_f,tr2_f,'k')
%legend("Referencia","GPC","GPC-FUZZY", "location", "northeastoutside");
%title("Trajetoria de referencia cinematica");

%Ajuste para o artigo
tr1_gpc_fuzzy = tr1_f;
tr2_gpc_fuzzy = tr2_f;
tr3_gpc_fuzzy = tr3_f;



ref = [x y z_];
xyz_pd = [tr1_pd' tr2_pd' tr3_pd'];
xyz_pd_fuzzy = [tr1_pd_fuzzy' tr2_pd_fuzzy' tr3_pd_fuzzy'];
xyz_gpc = [tr1_gpc' tr2_gpc' tr3_gpc'];
xyz_gpc_fuzzy = [tr1_gpc_fuzzy' tr2_gpc_fuzzy' tr3_gpc_fuzzy'];


inputs_pd = U_pd;
inputs_pd_fuzzy = U_pdf;
inputs_gpc = CTRL_GPC;
inputs_gpc_fuzzy = CTRL_FUZZ;


angles_pd = Q_pd;
angles_pd_fuzzy =  Q;
angles_gpc = Qc;
angles_gpc_fuzzy = Qc_f;


if track == "Circular"
  save "graficos_cir.mat" -mat7-binary track tr1_gpc_fuzzy tr2_gpc_fuzzy tr3_gpc_fuzzy ref ...
  xyz_pd xyz_pd_fuzzy xyz_gpc xyz_gpc_fuzzy inputs_pd inputs_pd_fuzzy inputs_gpc inputs_gpc_fuzzy...
  angles_pd  angles_pd_fuzzy angles_gpc angles_gpc_fuzzy
  
  
  elseif track == "Spiral"
    save "graficos_spi.mat" -mat7-binary track tr1_gpc_fuzzy tr2_gpc_fuzzy tr3_gpc_fuzzy ref ...
    xyz_pd xyz_pd_fuzzy xyz_gpc xyz_gpc_fuzzy inputs_pd inputs_pd_fuzzy inputs_gpc inputs_gpc_fuzzy...
    angles_pd  angles_pd_fuzzy angles_gpc angles_gpc_fuzzy  
    
  elseif track == "Sinusoidal"
    save "graficos_sin.mat" -mat7-binary track tr1_gpc_fuzzy tr2_gpc_fuzzy tr3_gpc_fuzzy ref ...
    xyz_pd xyz_pd_fuzzy xyz_gpc xyz_gpc_fuzzy inputs_pd inputs_pd_fuzzy inputs_gpc inputs_gpc_fuzzy...
    angles_pd  angles_pd_fuzzy angles_gpc angles_gpc_fuzzy  



  elseif track == "Triangular"
    save "graficos_tri.mat" -mat7-binary track tr1_gpc_fuzzy tr2_gpc_fuzzy tr3_gpc_fuzzy ref ...
    xyz_pd xyz_pd_fuzzy xyz_gpc xyz_gpc_fuzzy inputs_pd inputs_pd_fuzzy inputs_gpc inputs_gpc_fuzzy...
    angles_pd  angles_pd_fuzzy angles_gpc angles_gpc_fuzzy
    
  elseif track == "Circuit"
    save "graficos_circuit.mat" -mat7-binary track tr1_gpc_fuzzy tr2_gpc_fuzzy tr3_gpc_fuzzy ref ...
    xyz_pd xyz_pd_fuzzy xyz_gpc xyz_gpc_fuzzy inputs_pd inputs_pd_fuzzy inputs_gpc inputs_gpc_fuzzy...
    angles_pd  angles_pd_fuzzy angles_gpc angles_gpc_fuzzy
end























