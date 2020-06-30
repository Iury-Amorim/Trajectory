%SIMULACAO DE UM MANIPULADOR DE 3LINK

clc, clear -v, close all
pkg load symbolic
pkg load control
setenv PYTHON python





%% GERADOR DE TRAJETORIAS

Ts = 0.01;
operator = 1;

switch operator
    case 1 %% trajetoria circular 
        
    th = pi/2:Ts:5*pi/2;
    X =3 * cos(th); % deixar em 3
    Z =3 * sin(th); % deixar em 3
    z_c = (21+Z); % deixar 26/27
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
end





% -------cinematica inversa 3-links

  l1=12.8;
  l2=12.8;
  l3 = 8;
% for i=1:length(x)
%     beta(i) = atan2(y(i),x(i));
%     phi(i) = beta(i);
%     qsi(i) = -acos((x(i)^2+y(i)^2+l1^2-l2^2)/(2*l1*sqrt(x(i)^2+y(i)^2)));
%     theta1_pd(i) = (beta(i)+qsi(i));
%     theta2_pd(i) = acos((x(i)^2+y(i)^2-l1^2-l2^2)/(2*l1*l2));
%     theta3_pd(i) = -phi(i) + theta1_pd(i) + theta2_pd(i);
%     rho_pd(i) = atan2(z_(i),y(i));
% end



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

%ponto_inicial = [rho_pd(1)*ones(100,1) -theta1_pd(1)*ones(100,1) -theta2_pd(1)*ones(100,1) -theta3_pd(1)*ones(100,1)];

angles = [rho_pd' -theta1_pd' -theta2_pd' -theta3_pd'];

%angles = [ponto_inicial;angles_1];


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
%            if returnCode ~=0
%                disp('Failed connecting to remote API server')
%                break
%            end
            % get the joint velocity
            [returnCode, dq(ii)] = simxGetObjectFloatParameter(clientID,joint_handles(ii),2012, sim.simx_opmode_blocking); % parameter ID for angular velocity of the joint
%            if returnCode ~=0
%                disp('Failed connecting to remote API server')
%                break                
%            end
        end
        Q_pd = [Q_pd ; q];
        DQ = [DQ; dq];
        %        L = [l1  l2]*10^(-2); % arm segment lengths

%        %------- CONTROL LAW -----------------------


%        q = q;
%        dq = dq;
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
    
    % stop the simulation
%    simxStopSimulation(clientID, sim.simx_opmode_blocking);
    % Before closing the connection to V-REP,
    % make sure that the last command sent out had time to arrive.
%    simxGetPingTime(clientID);
    % Now close the connection to V-REP:
%    simxFinish(clientID)    ;
        
    
else
    disp('Failed connecting to remote API server');
end

% stop the simulation
%simxStopSimulation(clientID, sim.simx_opmode_blocking);

% Before closing the connection to V-REP,
% make sure that the last command sent out had time to arrive.
%simxGetPingTime(clientID);

% Now close the connection to V-REP:
%simxFinish(clientID)
disp('connection closed...')

%% Plot figure

%Plot error angles
Matix_erro1_pd = (rho_pd')    -Q_pd(2:end,1);
Matix_erro2_pd = (-theta1_pd')-Q_pd(2:end,2);
Matix_erro3_pd = (-theta2_pd')-Q_pd(2:end,3);
Matix_erro4_pd = (-theta3_pd')-Q_pd(2:end,4);

%figure
%error= angles - Q_pd(2:end,:)
%plot(error)

%Comparation angles
figure

subplot(4,1,1)
plot(rho_pd')
hold on
plot(Q_pd(2:end,1))

subplot(4,1,2)
plot(theta1_pd')
hold on
plot(-Q_pd(2:end,2))

subplot(4,1,3)
plot(theta2_pd')
hold on
plot(-Q_pd(2:end,3))

subplot(4,1,4)
plot(theta3_pd')
hold on
plot(-Q_pd(2:end,4))



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
    
##    if tr(1)>2*x(i) || tr(2)>2*y(i) || tr(3)>2*z_(i)
%  if tr(2)>2*y(i) || tr(3)>2*z_(i)
%        tr1(i) = x(i);
%        tr2(i) = y(i);
%        tr3(i) = z_(i);
%        cont = cont+1;
%    else
        tr1(i) = tr(1);
        tr2(i) = tr(2);
        tr3(i) = tr(3);
%    end
end



figure

plot3(x,z_,y)
hold on 
plot3(tr1,tr3,tr2,'r')


%% Ajuste para artigo

tr1_pd = tr1;
tr2_pd = tr2;
tr3_pd = tr3;
