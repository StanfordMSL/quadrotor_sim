function traj = traj_init(obj,model,t_end,mode)

x0 = obj.kf.x(:,1);
hz_fmu = model.clock.hz_fmu;

% Intermediate Variables
fmu_dt = 1/hz_fmu;
N      = hz_fmu*t_end + 1;

% Some Useful Parameters
traj.hz    = hz_fmu;
traj.t_fmu = 0:fmu_dt:t_end;
traj.x_bar = zeros(13,N);
traj.x_bar(:,1) = x0;
traj.k_N   = N;

switch mode
    case 'pos_att'
        % f_out: flat outputs
        traj.type = 'pos_att';
        
        traj.f_out = zeros(4,5,N);
    case 'body_rate'  
        % x: 10 state nominal (pos,vel,quat)
        % l: body rate input
        % L: feedback matrix (body rate)
        traj.type = 'body_rate';

        traj.x_br = zeros(10,N);
        traj.l_br = zeros(4,N-1);
        traj.L_br = zeros(4,10,N-1);
    case 'direct'
        % u: motor input
        traj.type = 'direct';
        
        traj.u_mt = zeros(4,N-1);
    case 'wrench'
        % u: wrench input
        traj.type = 'wrench';
        
        traj.u_wr = zeros(4,N-1);
end
