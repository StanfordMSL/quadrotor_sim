function A = A_calc_wrench(mu_curr,u_curr,model)

    dt = model.dt_ctl;

    % Quaternions
    q0 = mu_curr(7,1);
    q1 = mu_curr(8,1);
    q2 = mu_curr(9,1);
    q3 = mu_curr(10,1);
        
    % Omegas
    wx = mu_curr(11,1);
    wy = mu_curr(12,1);
    wz = mu_curr(13,1);
    
    % Inertia tensor
    Ixx = model.I_est(1,1);
    Iyy = model.I_est(2,2);
    Izz = model.I_est(3,3);

    A_pos =   [eye(3) dt*eye(3) zeros(3,7)];
    
    vel_c = (dt/model.m_est)*u_curr(1,1);
    vel_quat = vel_c*[( 2*q2) ( 2*q3) (2*q0) ( 2*q1);...
                      (-2*q1) (-2*q0) (2*q3) ( 2*q2);...
                      ( 4*q0) (    0) (   0)  (4*q3)];
    A_vel =   [zeros(3,3) eye(3) vel_quat zeros(3,3)];
    
    c   = 0.5*dt;
    Aq0 = c.*[1/c -wx -wy -wz -q1 -q2 -q3];
    Aq1 = c.*[ wx 1/c  wz -wy  q0 -q3  q2];
    Aq2 = c.*[ wy -wz 1/c  wx  q3  q0 -q1];
    Aq3 = c.*[ wz  wy -wx 1/c -q2  q1  q0];
    A_quat =  [zeros(4,6) [Aq0 ; Aq1 ; Aq2 ; Aq3]];
   
    omega_c = dt.*model.inv_I_est;
    Awx = (Izz-Iyy)*[0 wz wy];
    Awy = (Ixx-Izz)*[wz 0 wx];
    Awz = (Iyy-Ixx)*[wy wx 0];
    Aw_all = eye(3) - omega_c*[Awx ; Awy ; Awz];
    A_omega = [zeros(3,10) Aw_all];
    
    A = [A_pos ; A_vel ; A_quat ; A_omega];
 end