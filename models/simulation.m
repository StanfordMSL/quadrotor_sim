function log = simulation(traj,map,obj,model,targ,high_ctl,low_ctl)

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

% State Estimator Initilization
u_m = zeros(4,1);
x_est = traj.x(:,1);
sigma_est = zeros(13,13);

% Contact Parameter Initialization
dt_ct = 0.2;                        % Duration of Contact Force
n_ct  = 0;                          % Contact state (0 = no contact, 1 = contact/post-contact)
N_ct  = round(dt_ct*model.hz_act);  % Number of actual dynamics frames
FT_ext = zeros(6,1);

% Body Rate Controller Initialization
e_I = 0;
e_D_prev = 0;
w_hat = zeros(3,2);

for k_act = 1:N_sim
    t_now = (k_act-1)*dt_act;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % State Estimation Update
    if mod(t_now,dt_est) == 0
        k_est = k_est + 1;

        x_now = log.x_act(:,k_act);
        y = sensors(x_now,model);
        [x_est, sigma_est] = ekf(x_est,sigma_est,u_m,y,model);
        
        log.t_est(:,k_est)  = t_now;   
        log.x_est(:,k_est)  = x_est;        
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Low Level Control Updater    
    if (mod(t_now,dt_fmu) == 0) && (k_fmu < N_fmu-1)
        k_fmu = k_fmu + 1;
        
        switch low_ctl
            case 'fs_lqr_wrench'
                u_m = fs_lqr_wrench(x_now,traj.x(:,k_fmu),traj.u_w(:,k_fmu),traj.l(:,k_fmu),traj.L(:,:,k_fmu));  
            case 'br_ctrl'                
                % Generate Low Pass Filtered Body Rate
                w_hat = LPFilter(w_hat,x_now(11:13,1));
                
                % Generate Thrust Input
                thrust = thrust_ctrl(x_now,traj.f_out(:,:,k_fmu),model);
%                 thrust = traj.u_w(1,1);
                
                % Generate Body Rate Inputs
                w_des = traj.x(11:13,k_fmu);
                [u_tau,e_I,e_D_prev] = br_ctrl(x_now,w_des,e_I,w_hat,e_D_prev);
%                 u_tau = traj.u_w(2:4,1);
                
                % Output to Motors
                u_w = [thrust ; u_tau];               
                u_m = wrench2motor(u_w,model);
            case 'open_loop'
                u_w = traj.u_w(:,k_fmu);
                u_m = traj.u_m(:,k_fmu);
        end
        
        log.t_fmu(:,k_fmu) = t_now; 
        log.u_w(:,k_fmu)   = u_w;
        log.u_fmu(:,k_fmu) = u_m;
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
    
    % Simple Contact Model
    [FT_ext,n_ct,model,log] = contact_func(t_now,x_now,FT_ext,n_ct,model,targ,N_ct,log,'grasp');

    % Dynamic Model
    log.x_act(:,k_act+1) = quadcopter(log.x_act(:,k_act),u_m,model,FT_ext,'actual');
end
