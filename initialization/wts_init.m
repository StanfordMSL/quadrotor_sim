function wts = wts_init()

%% Control Weight Options %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Empty Weights %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
R_f = 1e-9*ones(1,1);
R_tau = 1e-6*ones(3,1);
R_vect = [R_f ; R_tau];
wts.R = diag(R_vect);   

%% State-Cost Weight Options %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Empty Weights %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Q_xy     = 0.001*ones(2,1);
Q_z      = 0.001*ones(1,1);
Q_vel    = 0.001*ones(3,1);
Q_q      = 0.001*[1 1 1 1]';
Q_omg_xy = 0.001*ones(2,1);
Q_omg_z  = 0.001*ones(1,1);

Q_vect = [Q_xy ; Q_z ; Q_vel ; Q_q ; Q_omg_xy ; Q_omg_z ];
wts.Q(:,:,1) = diag(Q_vect);

% Position Weighted %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Q_xy     = 1*ones(2,1);
Q_z      = 1*ones(1,1);
Q_vel    = 0*ones(3,1);
Q_q      = 0*[1 1 1 1]';
Q_omg_xy = 0*ones(2,1);
Q_omg_z  = 0*ones(1,1);

Q_vect = [Q_xy ; Q_z ; Q_vel ; Q_q ; Q_omg_xy ; Q_omg_z ];
wts.Q(:,:,2) = diag(Q_vect);

% Orientation Weighted %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Q_xy     = 1*ones(2,1);
Q_z      = 1*ones(1,1);
Q_vel    = 1*ones(3,1);
Q_q      = 10*[1 1 1 1]';
Q_omg_xy = 1e-9*ones(2,1);
Q_omg_z  = 1e-9*ones(1,1);

Q_vect = [Q_xy ; Q_z ; Q_vel ; Q_q ; Q_omg_xy ; Q_omg_z ];
wts.Q(:,:,3) = diag(Q_vect);

% Linear Velocity Weighted %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Q_xy     = 0*ones(2,1);
Q_z      = 0*ones(1,1);
Q_vel    = 1*ones(3,1);
Q_q      = 0*[1 1 1 1]';
Q_omg_xy = 0*ones(2,1);
Q_omg_z  = 0*ones(1,1);

Q_vect   = [Q_xy ; Q_z ; Q_vel ; Q_q ; Q_omg_xy ; Q_omg_z ];
wts.Q(:,:,4) = diag(Q_vect);

% Angular Velocity Weighted %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Q_xy     = 0*ones(2,1);
Q_z      = 0*ones(1,1);
Q_vel    = 1*ones(3,1);
Q_q      = 0*[1 1 1 1]';
Q_omg_xy = 0*ones(2,1);
Q_omg_z  = 0*ones(1,1);

Q_vect   = [Q_xy ; Q_z ; Q_vel ; Q_q ; Q_omg_xy ; Q_omg_z ];
wts.Q(:,:,5) = diag(Q_vect);

% Pose Weighted %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Q_xy     = 10*ones(2,1);
Q_z      = 10*ones(1,1);
Q_vel    = 1*ones(3,1);
Q_q      = 1*[1 1 1 1]';
Q_omg_xy = 1*ones(2,1);
Q_omg_z  = 1*ones(1,1);

Q_vect   = [Q_xy ; Q_z ; Q_vel ; Q_q ; Q_omg_xy ; Q_omg_z ];
wts.Q(:,:,6) = diag(Q_vect);

% Pose Extra Weighted %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Q_xy     = 100*ones(2,1);
Q_z      = 100*ones(1,1);
Q_vel    = 10*ones(3,1);
Q_q      = 100*[1 1 1 1]';
Q_omg_xy = 10*ones(2,1);
Q_omg_z  = 10*ones(1,1);

Q_vect   = [Q_xy ; Q_z ; Q_vel ; Q_q ; Q_omg_xy ; Q_omg_z ];
wts.Q(:,:,7) = diag(Q_vect);

% Pose Extra Extra Weighted %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Q_xy     = 1000*ones(2,1);
Q_z      = 1000*ones(1,1);
Q_vel    = 100*ones(3,1);
Q_q      = 10000*[1 1 1 1]';
Q_omg_xy = 100*ones(2,1);
Q_omg_z  = 100*ones(1,1);

Q_vect   = [Q_xy ; Q_z ; Q_vel ; Q_q ; Q_omg_xy ; Q_omg_z ];
wts.Q(:,:,8) = diag(Q_vect);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end
