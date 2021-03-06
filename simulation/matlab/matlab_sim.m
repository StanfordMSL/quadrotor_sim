function log = matlab_sim(traj,obj,model,high_ctl,low_ctl,sense_mode)

% Initialize clock variables
dt_lqr = model.clock.dt_lqr;
dt_ses = model.clock.dt_ses;
dt_fmu = model.clock.dt_fmu;
dt_act = model.clock.dt_act;

k_ses = 0;
k_lqr = 0;
k_fmu = 0;

% Initialize simulation final time and steps (fixed at 10kHz).
t_sim = dt_fmu*(size(traj.x,2)-1);
N_sim = round((t_sim/dt_act) + 1);
N_ses = round((t_sim/dt_ses) + 1);
N_fmu = round((t_sim/dt_fmu) + 1);

if floor(N_fmu) ~= N_fmu
    disp('[simulation]: Step check failed. Please choose an fmu rate that is a multiple of the simulation rate');
    log = 'error';
    return
end

% Initialize Logger Variable
log = logger_init(t_sim,N_sim,N_ses,N_fmu,traj,obj,model);     

% State Estimator Initilization
ses = ses_init(traj);

% Low Level Controller Initialization
u_mt = zeros(4,1);
switch low_ctl
    case 'pos_att'                
        pa = pa_init;
    case 'body_rate'                
        br = br_init();
    case 'wrench'
        % Do Nothing
    case 'direct'
        % Do Nothing
end

for k_act = 1:(N_sim-1)
    t_now = (k_act-1)*dt_act;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % State Estimation Update
    if mod(t_now,dt_ses) == 0
        k_ses = k_ses + 1;
        
        x_now = log.x_act(:,k_act);
        ses = state_estimation(x_now,u_mt,ses,sense_mode);
        
        log.t_ses(:,k_ses)   = t_now;   
        log.x_ses(:,k_ses)   = ses.x; 
        log.sigma(:,:,k_ses) = ses.sigma;
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Low Level Control Updater    
    if (mod(t_now,dt_fmu) == 0) && (k_fmu < N_fmu-1)
        k_fmu = k_fmu + 1;
        
        switch low_ctl
            case 'pos_att'
                f_out_now = traj.f_out(:,:,k_fmu);
                u_wr = pa_ctrl(ses.x,f_out_now,pa,model.est);
                
                % Output to Motors
                u_mt = wrench2motor(u_wr,model.est);
            case 'body_rate'  
                del_x = ses.x(1:10,:) - traj.x(1:10,k_fmu);

                u_op = traj.u_br(:,k_fmu);
                u_cl =  traj.L(:,:,k_fmu)*del_x;
        
                u_br = u_op +  u_cl;
                [u_wr ,br] = br_ctrl(ses.x,u_br,br);
                
                % Output to Motors
                u_mt = wrench2motor(u_wr,model.est);
            case 'direct'
                u_wr = traj.u_wr(:,k_fmu);
                u_mt = traj.u_mt(:,k_fmu);
            case 'wrench'
                u_wr = traj.u_wr(:,k_fmu);
                u_mt = traj.u_mt(:,k_fmu);
        end
        
        log.t_fmu(:,k_fmu) = t_now; 
        log.u_wr(:,k_fmu)  = u_wr;
        log.u_br(:,k_fmu)  = u_br;
        log.u_mt(:,k_fmu)  = u_mt;
        log.x_fmu(:,k_fmu) = x_now;
    end
    %%%%%%%%%%%%%%%%%%%%%%y%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % High Level Control Updater    
    if (mod(t_now,dt_lqr) == 0)
        k_lqr = k_lqr + 1;
        switch high_ctl
            case 'al_ilqr'
%                 tic
%                 traj = al_ilqr(traj,map,obj,model.est);
%                 toc
            case 'none'
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
    wt = model.ses.W*randn(13,1);
    log.x_act(:,k_act+1) = quadcopter_act(log.x_act(:,k_act),u_mt,FT_ext,wt);
end

% Tie Up Terminal Point
if k_ses < N_ses
    log.t_ses(:,end)  = t_now;   
    log.x_ses(:,end)  = ses.x;  
    log.x_fmu(:,end)  = ses.x;  
end

% Check Constraints
[log.con,~,~] = con_calc(log.x_fmu,log.u_br);
