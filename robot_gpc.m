%SIMULACAO DE UM MANIPULADOR DE 3LINK
clc, clear -x  tr1_pd tr2_pd tr3_pd tr1_pd_fuzzy tr2_pd_fuzzy tr3_pd_fuzzy ...
U_pd U Q_pd Q, close all
setenv PYTHON python
pkg load symbolic
pkg load control
pkg load signal

%% GERADOR DE TRAJETORIAS

Ts = 0.01;
operator = 2;

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
    figure (1)
    subplot(4,1,1)
    plot(rho')
    hold on
    plot(Qc(2:end,1))
    title("Rho-angulos")
    
    subplot(4,1,2)
    plot(-theta1')
    hold on
    plot(Qc(2:end,2))
    title("Theta1-angulos")
    
    
    subplot(4,1,3)
    plot(-theta2')
    hold on
    plot(Qc(2:end,3))
    title("Theta2-angulos")
    
    
    subplot(4,1,4)
    plot(-theta3')
    hold on
    plot(Qc(2:end,4))
    title("Theta3-angulos")
    
% Plot dos Sinais de Controle de cada junta
    figure(2)
    subplot(4,1,1)
    plot(sinal_matrix_dbg(4,10:end))
    title("Rho-sinal controle")
    subplot(4,1,2)
    plot(sinal_matrix_dbg(3,10:end))
     title("Theta1-sinal controle")
    subplot(4,1,3)
    plot(sinal_matrix_dbg(2,10:end))
     title("Theta2-sinal controle")
    subplot(4,1,4)
    plot(sinal_matrix_dbg(1,10:end))
    title("Theta3-sinal controle")

% Plot do Erro de cada junta
Matix_erro1 = (rho')    -Qc(2:end,1);
Matix_erro2 = (-theta1')-Qc(2:end,2);
Matix_erro3 = (-theta2')-Qc(2:end,3);
Matix_erro4 = (-theta3')-Qc(2:end,4);

     figure(3)
    subplot(4,1,1)
    plot(Matix_erro1)
    title("Rho-erro")
    
    subplot(4,1,2)
    plot(Matix_erro2)
     title("Theta1-erro")
     
    subplot(4,1,3)
    plot(Matix_erro3)
     title("Theta2-erro")
     
    subplot(4,1,4)
    plot(Matix_erro4)
    title("Theta3-erro")
    
    
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
   
%  if tr(2)>2*y(i) || tr(3)>2*z_(i)
%        tr1(i) = x(i);
%        tr2(i) = y(i);
%        tr3(i) = z_(i);
%        cont = cont+1;
%    else
        tr1(i) = tr(1);
        tr2(i) = tr(2);
        tr3(i) = tr(3);
    end
%end

figure (4)
plot3(x,-z_,y,'b');
hold on
plot3(-tr1,-tr3,tr2,'r')
legend("Referencia");
title("Trajetoria de referencia");


%% Ajuste para artigo
tr1_gpc = tr1;
tr2_gpc = tr2;
tr3_gpc = tr3;