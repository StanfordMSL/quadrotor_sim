function log = simulation(nom,wp,model,wts,targ,ctl_mode,mtr_mode)

% Initialize Logger Variable
log = logger_init(wp,model);     

dt_est = model.dt_est;
dt_lqr = model.dt_lqr;
dt_ctl = model.dt_ctl;
dt_fbc = model.dt_fbc;
dt_act = model.dt_act;

% We hardset the simulation to run at 10kHz
sim_N = wp.tf*model.hz_act;

k_est = 0;
k_fbc = 0;
k_ctl = 0;
k_lqr = 0;

%%% Contact Parameter Initialization
dt_ct = 0.2;                 % Duration of Contact Force
n_ct  = 0;                    % Contact state (0 = no contact, 1 = contact/post-contact)
N_ct  = round(dt_ct*model.hz_act); % Number of actual dynamics frames
FT_ext = zeros(6,1);

for k_act = 1:sim_N
    t_now = (k_act-1)*dt_act;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % State Estimator
    if mod(t_now,dt_est) == 0
        k_est = k_est + 1;

        % Perfect Sensing (used for flight control)
        x_now = log.x_act(:,k_act);
        log.x_fc(:,k_est)  = x_now;        
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Control Law Updater    
    if mod(t_now,dt_ctl) == 0
        k_ctl = k_ctl + 1;
        
        l = nom.l(:,:,k_ctl);
        L = nom.L(:,:,k_ctl);
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Feedback Controller Updater
    if mod(t_now,dt_fbc) == 0
        k_fbc = k_fbc + 1;
        
        x_bar = nom.x_bar(:,k_ctl);
        u_bar = nom.u_bar(:,k_ctl);
        alpha = nom.alpha;
        [u,curr_m_cmd] = fbc(x_now,x_bar,u_bar,l,L,alpha,model,mtr_mode);

        % Log State Control Commands
        log.m_cmd(:,k_fbc) = curr_m_cmd;  
        log.u(:,k_fbc)     = u;
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Control Updater    
    if mod(t_now,dt_lqr) == 0
        k_lqr = k_lqr + 1;
        switch ctl_mode
            case 'ilqr'
                [nom,cost] = ilqr_x(k_ctl,x_now,wp,nom,wts,model);
            case 'ileqr'
                [nom,cost] = ileqr_x(k_ctl,x_now,wp,nom,wts,model);
            case 'ileqr_oa'
                coeff_obs = [ 1.0 0.002;
                              1.1 0.002];
                [nom,cost] = ileqr_oa_x(k_ctl,x_now,wp,nom,wts,model,coeff_obs);    
            case 'al_ilqr'
                [nom,cost] = al_ilqr_x(k_ctl,x_now,wp,nom,wts,model);
            case 'df'
                cost = 0;
        end
        
        log.cost_curr(k_lqr,1) = cost;
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Dynamic Model
    
    % Simple Contact Model
    [FT_ext,n_ct,model,log] = contact_func(t_now,x_now,FT_ext,n_ct,model,targ,N_ct,log,'grasp');

    % Dynamic Model
    log.x_act(:,k_act+1) = quadcopter(log.x_act(:,k_act),curr_m_cmd,model,FT_ext,'actual');
end
