function log = matlab_sim(traj,obj,model,high_ctl,low_ctl,sense_mode)

% Initialize clock variables
dt_lqr = model.clock.dt_lqr;
dt_ses = model.clock.dt_ses;
dt_fmu = model.clock.dt_fmu;
dt_act = model.clock.dt_act;

k_act = 0;
k_ses = 0;
k_lqr = 0;
k_fmu = 0;

% Initialize simulation final time and steps (fixed at 10kHz).
t_sim = traj.t_fmu(end);
N_sim = round((t_sim/dt_act) + 1);
N_ses = round((t_sim/dt_ses) + 1);
N_fmu = round((t_sim/dt_fmu) + 1);

if floor(N_fmu) ~= N_fmu
    disp('[simulation]: Step check failed. Please choose an fmu rate that is a multiple of the simulation rate');
    log = 'error';
    return
end

% Initialize Logger Variable
log = logger_init(t_sim,N_sim,N_ses,N_fmu,traj,model);     

% State Estimator Initilization
ses = ses_init(obj,model);

% Low Level Controller Initialization
switch low_ctl
    case 'pos_att'                
        pa = pa_init();
    case 'body_rate'                
        % Do Nothing
    case 'wrench'
        % Do Nothing
    case 'direct'
        % Do Nothing
end

while (k_act < N_sim)
    k_act = k_act + 1;
    
    t_now = (k_act-1)*dt_act;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % State Estimation Update
    if mod(t_now,dt_ses) == 0
        k_ses = k_ses + 1;
        
        x_act = log.x_act(:,k_act);
        if k_fmu == 0
            x_bar = log.x_des(:,1);
        else
            x_bar = log.x_des(:,k_fmu);
        end
        ses = state_estimation(x_act,x_bar,ses,sense_mode);
        
        log.t_ses(:,k_ses)   = t_now;   
        log.x_ses(:,k_ses)   = ses.x;
        log.z_ses(:,k_ses)   = ses.z;
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
                u_mt = w2m(u_wr);
                
                % Pos Att Logging
                log.u_wr(:,k_fmu)  = u_wr;
            case 'body_rate'
                del_x = ses.s - traj.x_br(:,k_fmu);

                u_op = traj.u_br(:,k_fmu);
                u_cl = traj.L_br(:,:,k_fmu)*del_x;
                u_now = u_op + u_cl;
             
                % Body Rate Logging
                log.u_br(:,k_fmu)  = u_now;
            case 'wrench'
                u_wr = traj.u_wr(:,k_fmu);
                
                % Wrench Logging
                log.u_wr(:,k_fmu)  = u_wr;
        end
        
        % General Logging
        log.t_fmu(:,k_fmu) = t_now;
        log.x_fmu(:,k_fmu) = ses.x;        
    end
    %%%%%%%%%%%%%%%%%%%%%%y%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % High Level Control Updater    
    if (mod(t_now,dt_lqr) == 0) && (k_fmu < N_fmu-1)
        k_lqr = k_lqr + 1;
        switch high_ctl
            case 'al_ilqr'
%                 % Ciruit %%%
%                 tic
%                 obj = race_update(t_now,ses.x);
%                 traj_t = traj_init(obj,model,'body_rate');
%                 traj_t = diff_flat(obj,model,traj_t,'body_rate');
%                 nominal_plot(traj.x_bar,obj,20,'persp');
% 
%                 [traj_t,~] = al_ilqr(traj_t,obj,999);
%                 
%                 [traj_t,~] = al_ilqr(traj_t,obj,999);
%                 toc
                
%                 % Minimum Time %%%
%                 traj_t = trim(traj,k_fmu);
%                 traj_t = min_time_augment(traj_t,obj,ses.x,0);
%                 
%                 % Restitch
%                 traj = restitch(traj,traj_t,k_fmu);
% 
%                 % Update horizon
%                 N_fmu = size(traj.x_bar,2);
%                 N_sim = round(N_fmu*dt_fmu/dt_act);
                
            case 'none'
                % Carry on
        end
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Dynamic Model
    
    % Simple Contact/External Forces Model
    if strcmp(obj.type,'grasp') 
        [F_ext,obj,model] = contact_func(x_now,obj,model,'catch');
    else
        F_ext = zeros(3,1);
    end
    
    % Dynamic Model
    log.x_act(1:10,k_act+1) = quadcopter_act(log.x_act(1:10,k_act),u_now,F_ext);
    log.x_act(11:13,k_act)  = u_now(2:4,1);
    
    if log.x_act(7:10,k_act+1)'*log.x_act(7:10,k_act) < 0
        disp("UHOH... sign flip might need to be invoked");
    end
end

% Trim and Tie Up Terminal Point
log.t_act = 0:dt_act:(N_sim-1)*dt_act;
log.x_act = log.x_act(:,1:k_act);

log.t_ses  = log.t_ses(:,1:k_ses);
log.x_ses  = log.x_ses(:,1:k_ses);
log.sigma  = log.sigma(:,:,1:k_ses);

log.x_des = log.x_des;

log.t_fmu = log.t_fmu;
log.x_fmu = log.x_fmu;
log.u_br  = log.u_br(:,1:k_fmu);

for k = 2:N_fmu-1
    x  = log.x_fmu(1:10,k);
    u  = log.u_br(:,k);
    up = log.u_br(:,k-1);
    wm = 2000.*ones(4,1);
    
    log.u_mt(:,k) = br2wr(x,u,up,wm);            
end

% Check Constraints
% log.con = con_calc(log.x_fmu(1:10,:),log.u_br,obj.gt.p_box);
end