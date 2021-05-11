function log = simulation(traj,map,obj,model,high_ctl,low_ctl)

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
N_sim = round((t_sim/dt_act) + 1);
N_est = round((t_sim/dt_est) + 1);
N_fmu = round((t_sim/dt_fmu) + 1);

if floor(N_fmu) ~= N_fmu
    disp('[simulation]: Step check failed. Please choose an fmu rate that is a multiple of the simulation rate');
    log = 'error';
    return
end

% Initialize Logger Variable
log = logger_init(t_sim,N_sim,N_est,N_fmu,obj.x(:,1),model);     

% Store Desired Trajectory
log.x_des = traj.x;

% State Estimator Initilization
u_mt = zeros(4,1);
x_est = traj.x(:,1);
sigma_est = zeros(13,13);

% Body Rate Controller Initialization
switch low_ctl
    case 'pos_att'                
        pa = pa_init;
    case 'body_rate'                
        br = br_init();
    case 'direct'
        % Do Nothing
end

for k_act = 1:(N_sim-1)
    t_now = (k_act-1)*dt_act;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % State Estimation Update
    if mod(t_now,dt_est) == 0
        k_est = k_est + 1;

        x_now = log.x_act(:,k_act);
        x_est = x_now;
%         y = sensors(x_now,model);
%         [x_est, sigma_est] = ekf(x_est,sigma_est,u_mt,y,model);
        
        log.t_est(:,k_est)  = t_now;   
        log.x_est(:,k_est)  = x_est;        
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Low Level Control Updater    
    if (mod(t_now,dt_fmu) == 0) && (k_fmu < N_fmu-1)
        k_fmu = k_fmu + 1;
        
        switch low_ctl
            case 'pos_att'
                f_out_now = traj.f_out(:,:,k_fmu);
                u_wr = pa_ctrl(x_est,f_out_now,pa,model);
                
                % Output to Motors
                u_mt = wrench2motor(u_wr,model);
            case 'body_rate'                
                [u_wr ,br] = br_ctrl(x_estZ,u_br,br);
                
                % Output to Motors
                u_mt = wrench2motor(u_wr,model);
            case 'direct'
                u_wr = traj.u_w(:,k_fmu);
                u_mt = traj.u_m(:,k_fmu);
        end
        
        log.t_fmu(:,k_fmu) = t_now; 
        log.u_w(:,k_fmu)   = u_wr;
        log.u_fmu(:,k_fmu) = u_mt;
    end
    %%%%%%%%%%%%%%%%%%%%%%y%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % High Level Control Updater    
    if (mod(t_now,dt_lqr) == 0)
        k_lqr = k_lqr + 1;
        switch high_ctl
            case 'al_ilqr'
                tic
                traj = al_ilqr(traj,map,obj,model);
                toc
            case 'df'
                % Carry on
        end
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Dynamic Model
    
    % Simple Contact/External Forces Model
    if obj.type == 1
        [FT_ext,obj,model] = contact_func(x_now,obj,model,'catch');
    else
        FT_ext = zeros(6,1);
    end
    
    % Dynamic Model
    wt = model.W*randn(13,1);
    log.x_act(:,k_act+1) = quadcopter_act(log.x_act(:,k_act),u_mt,FT_ext,wt);
end

if k_est < N_est
    log.t_est(:,end)  = t_now;   
    log.x_est(:,end)  = x_est;  
end
