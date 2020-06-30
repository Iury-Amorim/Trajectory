%SIMULACAO DE UM MANIPULADOR DE 3LINK

clc, clear -x Q_pd track_xyz_pd tr1_pd tr2_pd tr3_pd Matix_erro1_pd Matix_erro2_pd Matix_erro3_pd Matix_erro4_pd, close all

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
    U_pd = [];
    U = [];
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
        %origin of system
##        [returnCode,origin] = simxGetObjectHandle(clientID,'PhantomXPincher_joint1', sim.simx_opmode_blocking);

        % get the angular position of the reference trajectory 
%         target_xyz = [x(i) y(i) z_(i)]
        % end effector
##        [returnCode,target_handle] = simxGetObjectHandle(clientID,'feltPen_tip', sim.simx_opmode_blocking);
%        [returnCode,target_handle] = simxGetObjectHandle(clientID,'feltPen_tip', sim.simx_opmode_blocking)

        % get the angular position of the reference trajectory 
##         [~,xyz_real] = simxGetObjectPosition(clientID,target_handle,-1,sim.simx_opmode_blocking); % retrieve absolute, not relative, position
##         track_xyz_pd = [track_xyz_pd  xyz_real];
        % get the angular position of the trajectory
%         thetas = [rho_pd(i) theta1_pd(i) theta2_pd(i) theta3_pd(i)]
        
        
%        if returnCode ~= 0
%            disp('Failed connecting to remote API server')
%            break
%        end
        

%          
%         track_xyz_pd_plot = xyz_real*100
%         target_xyz = [y(1) z_(1) x(1)]
        
%         if i== 1
%            offset = target_xyz(1,:) - track_xyz_pd_plot(1,:) 
%         end
% 
%         track_xyz_pd_plot = track_xyz_pd_plot + offset
        % plot(y,z_)
        % hold on 
        % plot(track_xyz_pd_plot(:,1),track_xyz_pd_plot(:,2))



   
        
%        q = zeros(1,length(joint_handles))
%        dq = zeros(1,length(joint_handles))    
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

        U = [U;u]; 
        U_pd = [U_pd;u_pd];
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
    simxStopSimulation(clientID, sim.simx_opmode_blocking);

    % Before closing the connection to V-REP,
    % make sure that the last command sent out had time to arrive.
    simxGetPingTime(clientID);

    % Now close the connection to V-REP:
    simxFinish(clientID) ;   
        
    
else
    disp('Failed connecting to remote API server')
end


% stop the simulation
simxStopSimulation(clientID, sim.simx_opmode_blocking);

% Before closing the connection to V-REP,
% make sure that the last command sent out had time to arrive.
simxGetPingTime(clientID);

% Now close the connection to V-REP:
simxFinish(clientID);
disp('connection closed...')

%% Plot figure
Matix_erro1_pdf = (rho')    -Q(2:end,1);
Matix_erro2_pdf = (-theta1')-Q(2:end,2);
Matix_erro3_pdf = (-theta2')-Q(2:end,3);
Matix_erro4_pdf = (-theta3')-Q(2:end,4);

figure
subplot(4,1,1)
plot(Matix_erro1_pdf ,'k')
hold on
plot(Matix_erro1_pd ,'r')
title(" Erro Rho")
legend('PDFUZZY','PD')

subplot(4,1,2)
plot(Matix_erro2_pdf ,'k')
hold on
plot(Matix_erro2_pd ,'r')
title(" Erro Theta1")
legend('PDFUZZY','PD')


subplot(4,1,3)
plot(Matix_erro3_pdf ,'k')
hold on
plot(Matix_erro3_pd ,'r')
title("Erro Theta2")
legend('PDFUZZY','PD')


subplot(4,1,4)
plot(Matix_erro4_pdf ,'k')
hold on
plot(Matix_erro4_pd ,'r')
title("Erro Theta3")
legend('PDFUZZY','PD')

%Plot error angles
%figure
%error= angles - Q(2:end,:)
%plot(error)

%Comparation angles

##pd_angulos;

figure
subplot(4,1,1)
plot(rho','k')
hold on
plot(Q(2:end,1),'b')
plot(Q_pd(2:end,1),'r')
title("Rho")
legend('REF','PDFUZZY','PD')

subplot(4,1,2)
plot(theta1','k')
hold on
plot(-Q(2:end,2),'b')
plot(-Q_pd(2:end,2),'r')
title("Theta1")
legend('REF','PDFUZZY','PD')


subplot(4,1,3)
plot(theta2','k')
hold on
plot(-Q(2:end,3),'b')
plot(-Q_pd(2:end,3),'r')
title("Theta2")
legend('REF','PDFUZZY','PD')


subplot(4,1,4)
plot(theta3','k')
hold on
plot(-Q(2:end,4),'b')
plot(-Q_pd(2:end,4),'r')
title("Theta3")
legend('REF','PDFUZZY','PD')


%% Control output

figure

subplot(4,1,1)
plot(U_f(11:end,1),'b');
hold on
plot(U_pd(11:end,1),'r');
legend('PDFUZZY','PD')
title("Rho")

subplot(4,1,2)
plot(U_f(11:end,2),'b');
hold on
plot(U_pd(11:end,2),'r');
legend('PDFUZZY','PD')
title("Theta1")

subplot(4,1,3)
plot(U_f(11:end,3),'b');
hold on
plot(U_pd(11:end,3),'r');
legend('PDFUZZY','PD')
title("Theta2")

subplot(4,1,4)
plot(U_f(11:end,4),'b');
hold on
plot(U_pd(11:end,4),'r');
legend('PDFUZZY','PD')
title("Theta3")



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
    
##    if tr(1)>2*x(i) || tr(2)>2*y(i) || tr(3)>2*z_(i)
%  if tr_F(2)>2*y(i) || tr_F(3)>2*z_(i)
%        tr1_F(i) = x(i);
%        tr2_F(i) = y(i);
%        tr3_F(i) = z_(i);
%        cont_F = cont_F+1;
%    else
        tr1_F(i) = tr_F(1);
        tr2_F(i) = tr_F(2);
        tr3_F(i) = tr_F(3);
%    end
end


figure
plot3(x,z_,y,'b')
hold on
plot3(tr1_pd,tr3_pd,tr2_pd,'r')
hold on
plot3(tr1_F,tr3_F,tr2_F,'k')
legend('REF','PD','PDFUZZY')

%% Ajuste para artigo

tr1_pd_fuzzy = tr1_F;
tr2_pd_fuzzy = tr2_F;
tr3_pd_fuzzy = tr3_F;



