 function B = B_calc(mu_curr,m,inv_I,m2w,dt)

    % Quaternions
    q0 = mu_curr(7,1);
    q1 = mu_curr(8,1);
    q2 = mu_curr(9,1);
    q3 = mu_curr(10,1);
    
    % nabla_pos 
    nabla_pos = zeros(3,4);
    
    % nabla_vel
    C1 = (1/m).*[2.*(q1*q3+q2*q0) ; 2.*(q2*q3-q1*q0) ; (q0^2-q1^2-q2^2+q3^2)];
%     C1 = (1/m).*[2.*(q1*q3+q2*q0) ; 2.*(q2*q3-q1*q0) ; (q0^2-q1^2-q2^2+q3^2)];
    nabla_vel = repmat(C1,1,4);
    
    % nabla_quat
    nabla_quat = zeros(4,4);
    
    % nabla_omega
    nabla_omega = inv_I*m2w(2:4,:);
    
    nabla_all = [nabla_pos ; nabla_vel ; nabla_quat ; nabla_omega];
    B = dt.*nabla_all;
    
%     % B for Position
%     B_pos = zeros(3,4);
%     
%     % B for Velocity
%     vel_vect  = (dt/m).*[2.*(q1*q3+q2*q0) ; 2.*(q2*q3-q1*q0) ; (q0^2-q1^2-q2^2+q3^2)];
% %     vel_vect =  (dt/m).*2.*[(q1*q3+q2*q0) ; (q2*q3-q1*q0) ; (q3^2+q0^2+0.5)];
%     B_vel     = vel_vect.*ones(3,4);
%     
%     % B for Quaternions
%     B_quat = zeros(4,4); 
%     
%     % B for Omegas
%     B_omega = dt.*inv_I*m2w(1:3,:); 
%     
%     % Combine the Bs
%     B = [B_pos ; B_vel ; B_quat ; B_omega];
 end