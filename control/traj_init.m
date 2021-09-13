function traj = traj_init(obj,model,mode)

% Unpack some stuff
x0 = obj.kf.x(:,1);
x1 = obj.kf.x(:,2);
hz_fmu = model.clock.hz_fmu;

% Estimate t_end using assumed velocity.
t_end = round(norm(x1(1:3)-x0(1:3))/model.misc.v_cr,1);

% Intermediate Variables
fmu_dt = 1/hz_fmu;
N      = round(hz_fmu*t_end + 1);
u_br_hov = [f2fn(model.motor.thrust_hover) ; 0 ; 0 ; 0];

% Some Useful Parameters
traj.hz    = hz_fmu;
traj.t_fmu = 0:fmu_dt:t_end;

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

        traj.x_br = repmat([x0(:,1) ; zeros(7,1)] ,1,N);
        traj.u_br = repmat(u_br_hov,1,N-1);
        traj.L_br = zeros(4,17,N-1);
    case 'direct'
        % u: motor input
        traj.type = 'direct';
        
        traj.u_mt = zeros(4,N-1);
    case 'wrench'
        % u: wrench input
        traj.type = 'wrench';
        
        traj.u_wr = zeros(4,N-1);
end
