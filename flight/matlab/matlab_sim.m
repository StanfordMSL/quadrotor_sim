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
log = logger_init(t_sim,N_sim,N_ses,N_fmu,traj,obj,model);     

% State Estimator Initilization
ses = ses_init(obj,model);

% Low Level Controller Initialization
u_mt = zeros(4,1);
switch low_ctl
    case 'pos_att'                
        pa = pa_init();
    case 'body_rate'                
        br = br_init();
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
                
                [u_wr,br] = br_ctrl(ses.x,u_now,br);
                
                % Output to Motors
                u_mt = w2m(u_wr);
                
                % Body Rate Logging
                log.u_wr(:,k_fmu)  = u_wr;
                log.u_br(:,k_fmu)  = u_now;
            case 'direct'
                u_mt = traj.u_mt(:,k_fmu);
            case 'wrench'
                u_wr = traj.u_wr(:,k_fmu);
                u_mt = w2m(u_wr);
                
                % Wrench Logging
                log.u_wr(:,k_fmu)  = u_wr;
        end
        
        % General Logging
        log.t_fmu(:,k_fmu) = t_now;
        log.x_fmu(:,k_fmu) = ses.x;
        log.u_mt(:,k_fmu)  = u_mt;
        
        ses.u = u_mt;
    end
    %%%%%%%%%%%%%%%%%%%%%%y%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % High Level Control Updater    
    if (mod(t_now,dt_lqr) == 0) && (k_fmu < N_fmu-1)
        k_lqr = k_lqr + 1;
        switch high_ctl
            case 'al_ilqr'
%                 [traj,t_end] = min_time_augment(traj,obj,k_fmu,0);
%                 N_sim = round((t_end/dt_act) + 1);
%                 N_fmu = round((t_end/dt_fmu) + 1);
            case 'none'
                % Carry on
        end
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Dynamic Model
    
    % Simple Contact/External Forces Model
    if strcmp(obj.type,'grasp') 
        [FT_ext,obj,model] = contact_func(x_now,obj,model,'catch');
    else
        FT_ext = zeros(6,1);
    end
    
    % Dynamic Model
    wt = model.ses.W*randn(13,1);
    log.x_act(:,k_act+1) = quadcopter_act(log.x_act(:,k_act),u_mt,FT_ext,wt);
    
    if log.x_act(7:10,k+1)'*log.x_act(7:10,k) < 0
        disp("UHOH... sign flip might need to be invoked");
    end
end

% Trim and Tie Up Terminal Point
log.t_act = log.t_act(:,1:k_act);
log.x_act = log.x_act(:,1:k_act);

log.t_ses  = log.t_ses(:,1:k_ses);
log.x_ses  = log.x_ses(:,1:k_ses);
log.sigma  = log.sigma(:,:,1:k_ses);

log.x_des = log.x_des;

log.t_fmu = log.t_fmu(:,1:k_fmu);
log.x_fmu = log.x_fmu(:,1:k_fmu);
log.u_wr  = log.u_wr(:,1:k_fmu-1);
log.u_br  = log.u_br(:,1:k_fmu-1);
log.u_mt  = log.u_mt(:,1:k_fmu-1);


% Check Constraints
% log.con = con_calc(log.x_fmu(1:10,:),log.u_br,obj.gt.p_box);
end