function fc = fc_init(model,type)

switch type
    case 'ilqr'
        fc.type = 'ilqr';
        
        % Stagewise R
        R_f = 0*ones(1,1);
        R_tau = 0*ones(3,1);
        R_vect = [R_f ; R_tau];
        fc.R = diag(R_vect);   
        
        fc.Q = zeros(13,13,1);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%               
        % Empty Cost
        Q_xy     = 0*ones(2,1);
        Q_z      = 0*ones(1,1);
        Q_vel    = 0*ones(3,1);
        Q_q      = 0*[1 1 1 1]';
        Q_omg_xy = 0*ones(2,1);
        Q_omg_z  = 0*ones(1,1);
        
        Q_vect = [Q_xy ; Q_z ; Q_vel ; Q_q ; Q_omg_xy ; Q_omg_z ];
        fc.Q(:,:,1) = diag(Q_vect);
        
        % Position Weighted Q
        Q_xy     = 1*ones(2,1);
        Q_z      = 1*ones(1,1);
        Q_vel    = 0*ones(3,1);
        Q_q      = 1*[1 1 1 1]';
        Q_omg_xy = 0*ones(2,1);
        Q_omg_z  = 0*ones(1,1);
        
        Q_vect = [Q_xy ; Q_z ; Q_vel ; Q_q ; Q_omg_xy ; Q_omg_z ];
        fc.Q(:,:,2) = diag(Q_vect);
        
        % Orientation Weighted Q
        Q_xy     = 0*ones(2,1);
        Q_z      = 0*ones(1,1);
        Q_vel    = 0*ones(3,1);
        Q_q      = 1*[1 1 1 1]';
        Q_omg_xy = 0*ones(2,1);
        Q_omg_z  = 0*ones(1,1);
        
        Q_vect = [Q_xy ; Q_z ; Q_vel ; Q_q ; Q_omg_xy ; Q_omg_z ];
        fc.Q(:,:,3) = diag(Q_vect);
        
        % Velocity Weighted Q
        Q_xy     = 0*ones(2,1);
        Q_z      = 0*ones(1,1);
        Q_vel    = 1*ones(3,1);
        Q_q      = 0*[1 1 1 1]';
        Q_omg_xy = 0*ones(2,1);
        Q_omg_z  = 0*ones(1,1);
        
        Q_vect   = [Q_xy ; Q_z ; Q_vel ; Q_q ; Q_omg_xy ; Q_omg_z ];
        fc.Q(:,:,4) = diag(Q_vect);
        
        % Omega Weighted Q
        Q_xy     = 10*ones(2,1);
        Q_z      = 10*ones(1,1);
        Q_vel    = 1*ones(3,1);
        Q_q      = 10*[1 1 1 1]';
        Q_omg_xy = 1*ones(2,1);
        Q_omg_z  = 1*ones(1,1);
        
        Q_vect   = [Q_xy ; Q_z ; Q_vel ; Q_q ; Q_omg_xy ; Q_omg_z ];
        fc.Q(:,:,5) = diag(Q_vect);
        
        % Pose Weighted Q
        Q_xy     = 10*ones(2,1);
        Q_z      = 10*ones(1,1);
        Q_vel    = 1*ones(3,1);
        Q_q      = 1*[1 1 1 1]';
        Q_omg_xy = 1*ones(2,1);
        Q_omg_z  = 1*ones(1,1);
        
        Q_vect   = [Q_xy ; Q_z ; Q_vel ; Q_q ; Q_omg_xy ; Q_omg_z ];
        fc.Q(:,:,6) = diag(Q_vect);
        
        % Pose Extra Weighted Q
        Q_xy     = 10*ones(2,1);
        Q_z      = 10*ones(1,1);
        Q_vel    = 10*ones(3,1);
        Q_q      = 10*[1 1 1 1]';
        Q_omg_xy = 10*ones(2,1);
        Q_omg_z  = 10*ones(1,1);
        
        Q_vect   = [Q_xy ; Q_z ; Q_vel ; Q_q ; Q_omg_xy ; Q_omg_z ];
        fc.Q(:,:,7) = diag(Q_vect);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Inversion Guaranteeing Offset
        fc.rho = 1;
    otherwise
        % Do Nothing
end
end
