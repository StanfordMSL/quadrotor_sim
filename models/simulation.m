function log = simulation(traj,obj,wts_db,model,targ,ctl_mode)

% Initialize clock variables
dt_lqr = model.dt_lqr;
dt_est = model.dt_est;
dt_fmu = model.dt_fmu;
dt_act = model.dt_act;

k_est = 0;
k_lqr = 0;
k_fmu = 0;

% Initialize simulation final time and steps (fixed at 10kHz).
t_sim = dt_fmu*(size(traj.x,2)-1);
N_sim = (t_sim/dt_act) + 1;
N_est = (t_sim/dt_est) + 1;
N_fmu = (t_sim/dt_fmu) + 1;

if floor(N_fmu) ~= N_fmu
    disp('[simulation]: Step check failed. Please choose an fmu rate that is a multiple of the simulation rate');
    log = 'error';
    return
end

% Initialize Logger Variable
log = logger_init(t_sim,N_sim,N_est,N_fmu,obj.wp_arr(:,1),model);     

% Contact Parameter Initialization
dt_ct = 0.2;                        % Duration of Contact Force
n_ct  = 0;                          % Contact state (0 = no contact, 1 = contact/post-contact)
N_ct  = round(dt_ct*model.hz_act);  % Number of actual dynamics frames
FT_ext = zeros(6,1);

for k_act = 1:N_sim
    t_now = (k_act-1)*dt_act;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % State Estimation Update
    if mod(t_now,dt_est) == 0
        k_est = k_est + 1;

        % Perfect Sensing (used for flight control)
        x_now = log.x_act(:,k_act);
        log.t_est(:,k_est)  = t_now;   
        log.x_est(:,k_est)  = x_now;        
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Controller Update    
    if (mod(t_now,dt_fmu) == 0) && (k_fmu < N_fmu-1)
        k_fmu = k_fmu + 1;
        
%         l = traj.l(:,k_fmu);
        L = traj.L(:,:,k_fmu);
        
        x_bar = traj.x(:,k_fmu);
        u_bar = traj.u(:,k_fmu);
        
        del_x = x_now - x_bar;
        del_u = L*del_x;
        u_now = u_bar + del_u;
        
        log.t_fmu(:,k_fmu)  = t_now; 
        log.u_fmu(:,k_fmu)  = u_now;
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % iLQR Updater    
    if (mod(t_now,dt_lqr) == 0)
        k_lqr = k_lqr + 1;
        switch ctl_mode
            case 'msl_lqr'
                tic
                traj = msl_lqr(k_fmu,traj,obj,wts_db,model);
                toc
            case 'df'
                % Carry on
        end
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Dynamic Model
    
    % Simple Contact Model
    [FT_ext,n_ct,model,log] = contact_func(t_now,x_now,FT_ext,n_ct,model,targ,N_ct,log,'grasp');

    % Dynamic Model
    log.x_act(:,k_act+1) = quadcopter(log.x_act(:,k_act),u_now,model,FT_ext,'actual');
end
