function traj = traj_init(mode)

% Tunable Parameters
traj.hz = 100;
traj.t_end =10;

N = traj.hz*traj.t_end + 1;

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
