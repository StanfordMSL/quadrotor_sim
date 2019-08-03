function C = C_calc(mu_curr,model)
    % Quaternions
    q0 = mu_curr(7,1);
    q1 = mu_curr(8,1);
    q2 = mu_curr(9,1);
    q3 = mu_curr(10,1);
   
    C_cam  = [eye(3) zeros(3,10)];
    C_baro = [zeros(1,2) 1 zeros(1,10)];

    % note: we ignore the k_d * wRb * vel component! Fill in if you have time.
%     temp = -model.g .* [  ( 2*q2)  (2*q3)  (2*q0) (2*q1) ;
%                           (-2*q1) (-2*q0)  (2*q3) (2*q2) ;
%                            (4*q0)  (   0)  (   0) (4*q3)];
    temp = -model.g .* [  (-2*q2)  (2*q3) (-2*q0) (2*q1) ;
                          ( 2*q1)  (2*q0)  (2*q3) (2*q2) ;
                           (4*q0)  (   0)  (   0) (4*q3)];
    C_acc = [zeros(3,6) temp zeros(3,3)];
    
%     C_theta_mag = [zeros(1,6) (-2*q3) (2*q2) (2*q1) (-2*q0) zeros(1,3)];
%     C_phi_mag   = [zeros(1,6) ( 4*q0)    (0) (4*q2)     (0) zeros(1,3)];
%     C_psi_mag   = [zeros(1,6)  (2*q1) (2*q0) (2*q3)  (2*q2) zeros(1,3)];
    C_theta_mag = [zeros(1,6) ( 2*q3)   (2*q2) (2*q1) ( 2*q0) zeros(1,3)];
    C_phi_mag   = [zeros(1,6) ( 4*q0)      (0) (4*q2)     (0) zeros(1,3)];
    C_psi_mag   = [zeros(1,6)  (-2*q1) (-2*q0) (2*q3)  (2*q2) zeros(1,3)];
    C_mag = [C_theta_mag ; C_phi_mag ; C_psi_mag];
    
    C_gyro = [zeros(3,10) eye(3)];
    
    C = [C_cam ; C_baro ; C_acc ; C_mag ; C_gyro];
%     C = [C_gyro];
end