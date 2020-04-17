function log = simulation(wp,model,wts,targ,ws_mode,ctl_mode)

% Initialize Logger Variable
log = logger_init(wp,model.dt_act,model.dt_fbc);     

dt_est = model.dt_est;
dt_lqr = model.dt_lqr;
dt_ctl = model.dt_ctl;
dt_fbc = model.dt_fbc;
dt_act = model.dt_act;

% We hardset the simulation to run at 10kHz
sim_N = wp.tf*model.hz_act;

k_est = 1;
k_fbc = 0;
k_ctl = 0;

%%% Contact Parameter Initialization
k_ct  = 1;                   % Contact Force Time Counter
st_ct = 0;                   % Contact state (0 = no contact, 1 = contact/post-contact)
dt_ct = 0.2;                 % Duration of Contact Force
N_ct  = round(dt_ct*model.hz_act); % Number of actual dynamics frames

% Warm Start with either differential flatness or iLQR itself
switch ws_mode
    case 'df'
        nom = df_init(wp,model,'yaw','hide');
    case 'ilqr'
        nom = ilqr_init(wp,wts,model);
end

for k_act = 1:sim_N
    t_now = (k_act-1)*dt_act;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % State Estimator
    if mod(t_now,dt_est) == 0
        % Perfect Sensing (used for flight control)
        x_now = log.x_act(:,k_act);
        log.x_fc(:,k_est)  = x_now;
        
        k_est = k_est + 1;
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
        [u,curr_m_cmd] = fbc(x_now,x_bar,u_bar,l,L,alpha,model,'actual');

        % Log State Control Commands
        log.m_cmd(:,k_fbc) = curr_m_cmd;  
        log.u(:,k_fbc)     = u;
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % LQR Updater    
    if mod(t_now,dt_lqr) == 0
        switch ctl_mode
            case 'ilqr'
                nom = ilqr_x(k_ctl,x_now,wp,nom,wts,model);
            case 'ileqr'
                nom = ileqr_x(k_ctl,x_now,wp,nom,wts,model);
        end
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Dynamic Model
    check_ct = targ.pos - log.x_act(1:3,k_act);

    if (norm(check_ct) < model.leg_l) && (st_ct == 0)
        [FT_ext_arr,model,targ] = contact_func(model,targ,t_act(k_act),log.x_act(:,k_act),check_ct,N_ct);

        FT_ext = FT_ext_arr(:,k_ct);
        k_ct = k_ct + 1;

        st_ct = 1;

        disp('[main]: Contact triggered!');
    elseif (k_ct <= N_ct) && (st_ct == 1)
            FT_ext = FT_ext_arr(:,k_ct);
            k_ct = k_ct + 1;
    else
        FT_ext = zeros(6,1);
    end

    log.x_act(:,k_act+1) = quadcopter(log.x_act(:,k_act),curr_m_cmd,model,FT_ext,'actual');
end
