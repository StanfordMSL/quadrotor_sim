function fc = fc_init(model,type)

switch type
    case 'tvlqr'
        fc.type = 'tvlqr';
        Q_xy  = 10*ones(2,1);
        Q_z   = 20*ones(1,1);
        Q_xy_dot = 1.6*ones(3,1);
        Q_q   = 1000*ones(4,1);
        Q_omg_xy = 1.1*ones(2,1);
        Q_omg_z = 2*ones(1,1);

        Q_vect = [Q_xy ; Q_z ; Q_xy_dot ; Q_q ; Q_omg_xy ; Q_omg_z ];
        fc.Q = diag(Q_vect);

        fc.R = 1e-2*eye(4);
        
        fc.k_fr = 1;
        fc.k_kfr = 1;
    case 'tilqr'
        fc.type = 'tilqr';
        
        Q_xy  = 10*ones(2,1);
        Q_z   = 10*ones(1,1);
        Q_vel = 1.6*ones(3,1);
        Q_q   = 10*ones(3,1);
        Q_omg_xy = 1.1*ones(2,1);
        Q_omg_z = 2*ones(1,1);

        Q_vect = [Q_xy ; Q_z ; Q_vel ; Q_q ; Q_omg_xy ; Q_omg_z ];
        Q = diag(Q_vect);

        R = 1e-1*eye(4);
        
        mu_init = zeros(12,1);
        hover = sqrt((model.m_act*model.g)/(4*model.kt_act(1,1)));
        u_init = hover.*ones(4,1);

        u2w = A_calc(mu_init,u_init,model);   
        B = B_calc(mu_init,u_init,model);

        rank(ctrb(u2w,B))
        [fc.K,~,~] = dlqr(u2w,B,Q,R);
    case 'ilqr'
        fc.type = 'ilqr';
        
        % Stagewise R
        R_f = 1e-5*ones(1,1);
        R_tau = 1e-5*ones(3,1);
        R_vect = [R_f ; R_tau];
        fc.R = diag(R_vect);   
        
        fc.Q = zeros(12,12,4);
               
        % Stagewise Q
        Q_xy     = 1*ones(2,1);
        Q_z      = 1*ones(1,1);
        Q_vel    = 1*ones(3,1);
        Q_q      = 1*[100 100 1]';
        Q_omg_xy = 1*ones(2,1);
        Q_omg_z  = 1*ones(1,1);
        
        Q_vect = [Q_xy ; Q_z ; Q_vel ; Q_q ; Q_omg_xy ; Q_omg_z ];
        fc.Q(:,:,1) = diag(Q_vect);
        
        % Position Weighted Q
        Q_xy     = 100*ones(2,1);
        Q_z      = 100*ones(1,1);
        Q_vel    = 0*ones(3,1);
        Q_q      = 0*[1 1 1]';
        Q_omg_xy = 0*ones(2,1);
        Q_omg_z  = 0*ones(1,1);
        
        Q_vect = [Q_xy ; Q_z ; Q_vel ; Q_q ; Q_omg_xy ; Q_omg_z ];
        fc.Q(:,:,2) = diag(Q_vect);
        
        % Orientation Weighted Q
        Q_xy     = 5*ones(2,1);
        Q_z      = 5*ones(1,1);
        Q_vel    = 1*ones(3,1);
        Q_q      = [10 10 10]';
        Q_omg_xy = 1*ones(2,1);
        Q_omg_z  = 1*ones(1,1);
        
        Q_vect = [Q_xy ; Q_z ; Q_vel ; Q_q ; Q_omg_xy ; Q_omg_z ];
        fc.Q(:,:,3) = diag(Q_vect);
        
        % Pose Weighted Q
        Q_xy     = 10*ones(2,1);
        Q_z      = 10*ones(1,1);
        Q_vel    = 1*ones(3,1);
        Q_q      = 10*[1 1 1]';
        Q_omg_xy = 1*ones(2,1);
        Q_omg_z  = 1*ones(1,1);
        
        Q_vect   = [Q_xy ; Q_z ; Q_vel ; Q_q ; Q_omg_xy ; Q_omg_z ];
        fc.Q(:,:,4) = diag(Q_vect);
        
        % Terminal Q
        fc.Q_N = diag(Q_vect);
        disp('[fc_init]: TODO: implement a cleaner way of holding Q_N. It is not in the same array as the rest');
        
    case 'PID'
        fc.type = 'PID';
        % Pitch/Roll Gains
        fc.att_xy_kP = 20;
        fc.att_xy_kI = 15;
        fc.att_xy_kD = 10;

        % Yaw Gains Gains
        fc.att_z_kP = 30;
        fc.att_z_kI = 5;
        fc.att_z_kD = 2;

        % Altitude Gains
        fc.zkP = 900;
        fc.zkI = 600;
        fc.zkD = 200;

        % Pos XY Gains
        fc.xykP = 1;
        fc.xykI = 0.2;
        fc.xykD = 0.6;

        fc.err_xy_limit = 100;

        % Error Structure
        fc.err_P_pos = 0;
        fc.err_P_pos_prior = 0;
        fc.err_I_pos = 0;
        fc.err_I_pos_max = 10;

        fc.err_P_att = zeros(3,1);
        fc.err_P_att_prior = zeros(3,1);
        fc.err_I_att = zeros(3,1);
        fc.err_I_att_max = 10;

        fc.err_dt = 1./model.fc_hz;
    otherwise
        % Do Nothing
end

disp('[controller_init]: Controller uses a kw^2 estimate. Could be better (LS).');
end
