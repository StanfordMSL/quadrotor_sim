function wts = wts_init()

%% Control Weight Options %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Negligible Weights %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
R_f     = 1e-9*ones(1,1);
R_tau   = 1e-9*ones(3,1);
R_dt    = 1*ones(1,1);

R_vect      = [R_f ; R_tau ; R_dt];
wts.R_stnd  = diag(R_vect);   

%% State-Cost Weight Options %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Position Weighted %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Q_xy     = 1.*ones(2,1);
Q_z      = 1.*ones(1,1);
Q_vel    = 1.*ones(3,1);
Q_q      = 1.*[1 1 1 1]';
Q_omg_xy = 1.*ones(2,1);
Q_omg_z  = 1.*ones(1,1);

Q_vect    = [Q_xy ; Q_z ; Q_vel ; Q_q ; Q_omg_xy ; Q_omg_z];
wts.Q_pstn = diag(Q_vect);

% Pose Weighted %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Q_xy     = 1.*ones(2,1);
Q_z      = 1.*ones(1,1);
Q_vel    = 0.*ones(3,1);
Q_q      = 1.*[1 1 1 1]';
Q_omg_xy = 0.*ones(2,1);
Q_omg_z  = 0.*ones(1,1);

Q_vect    = [Q_xy ; Q_z ; Q_vel ; Q_q ; Q_omg_xy ; Q_omg_z];
wts.Q_pose = diag(Q_vect);

% Uniform Weights %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Q_xy     = 1.*ones(2,1);
Q_z      = 1.*ones(1,1);
Q_vel    = 1.*ones(3,1);
Q_q      = 1.*ones(4,1);
Q_omg_xy = 1.*ones(2,1);
Q_omg_z  = 1.*ones(1,1);

Q_vect    = [Q_xy ; Q_z ; Q_vel ; Q_q ; Q_omg_xy ; Q_omg_z];
wts.Q_unif = diag(Q_vect);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end
