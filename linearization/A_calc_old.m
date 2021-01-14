function A = A_calc(x_curr,u_curr,m,I,inv_I,kd,D,dt)

    % Quaternions
    q0 = x_curr(7,1);
    q1 = x_curr(8,1);
    q2 = x_curr(9,1);
    q3 = x_curr(10,1);
    quat = x_curr(7:10,1);
    
    % Omegas
    wx = x_curr(11,1);
    wy = x_curr(12,1);
    wz = x_curr(13,1);
        
    % Inertia tensor
    Ixx = I(1,1);
    Iyy = I(2,2);
    Izz = I(3,3);

    % nabla_pos 
    J_pos = [zeros(3,3) eye(3) zeros(3,7)];
    
    % nabla_vel
    % MISSING THE U VALUE INPUTS
    disp('MISSING U VALUE IN THE PARTIALS');
    % NEED TO ADD THE PARTIALS FROM DRAG (qDq*)v
    A1 = -quatrotate(quatconj(quat'),D)';
    A2 = [( 2*q2) ( 2*q3) ( 2*q0) ( 2*q1);...
          (-2*q1) (-2*q0) ( 2*q3) ( 2*q2);...
          ( 2*q0) (-2*q1) (-2*q2) ( 2*q3)];
      
%     A2 = [( 2*q2) ( 2*q3) ( 2*q0) ( 2*q1);...
%           (-2*q1) (-2*q0) ( 2*q3) ( 2*q2);...
%           ( 4*q0) (    0) (   0)  (4*q3)];
    J_vel = [zeros(3,3) A1 A2 zeros(3,3)];
    
    % nabla_quat
    B1 = 0.5.*[ 0 -wx -wy -wz -q1 -q2 -q3 ;... 
               wx   0  wz -wy  q0 -q3  q2 ;...
               wy -wz   0  wx  q3  q0 -q1 ;...
               wz  wy -wx   0 -q2  q1  q0 ];
    nabla_quat = [zeros(4,6) B1];
    
    % nabla_omega
    % NEED TO ADD THE PARTIALS FROM A and B DRAG (qDq*)v
    C1 = -(Izz-Iyy)*[0 wz wy];
    C2 = -(Ixx-Izz)*[wz 0 wx];
    C3 = -(Iyy-Ixx)*[wy wx 0];
    C_all = inv_I*[C1 ; C2 ; C3];
    nabla_omega = [zeros(3,10) C_all];
    
    nabla_all = [J_pos ; J_vel ; nabla_quat ; nabla_omega];
    A = eye(13) + dt.*nabla_all;
    
%     %%%
%     A_pos  = [eye(3) dt*eye(3) zeros(3,7)];
%     
%     % A for Velocity
%     vel_c = (dt/m)*sum(u_curr);
%     vel_C = (kd.*dt./m).*eye(3);
%     vel_quat = vel_c*[( 2*q2) ( 2*q3) ( 2*q0) ( 2*q1);...
%                       (-2*q1) (-2*q0) ( 2*q3) ( 2*q2);...
%                       ( 2*q0) (-2*q1) (-2*q2) ( 2*q3)];
%     A_vel =   [zeros(3,3) (eye(3)-vel_C) vel_quat zeros(3,3)];
%     
%     % A for Quaternions
%     c   = 0.5*dt;
%     Aq0 = c.*[1/c -wx -wy -wz -q1 -q2 -q3];
%     Aq1 = c.*[ wx 1/c  wz -wy  q0 -q3  q2];
%     Aq2 = c.*[ wy -wz 1/c  wx  q3  q0 -q1];
%     Aq3 = c.*[ wz  wy -wx 1/c -q2  q1  q0];
%     A_quat =  [zeros(4,6) [Aq0 ; Aq1 ; Aq2 ; Aq3]];
%    
%     % A for Omegas
%     omega_c = dt.*inv_I;
%     Awx = (Izz-Iyy)*[0 wz wy];
%     Awy = (Ixx-Izz)*[wz 0 wx];
%     Awz = (Iyy-Ixx)*[wy wx 0];
%     Aw_all = eye(3) - omega_c*[Awx ; Awy ; Awz];
%     A_omega = [zeros(3,10) Aw_all];
% 
%     % Combine the non-dt As
%     A = [A_pos ; A_vel ; A_quat ; A_omega];
    
 end