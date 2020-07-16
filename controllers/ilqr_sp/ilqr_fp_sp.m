function [x_bar,u_bar,J,c_con] = ilqr_fp_sp(x_bar,u_bar,l,L,x_star,al,wts,model,init_flag)

    % Unpack and define some useful stuff
    Q_t = wts.Q_pstn;
    Q_f = wts.Q_unif;
    R   = wts.R_stnd;
    FT_ext = zeros(6,1);

    % Forward Pass Counter Variables
    N = model.N_ctl + 1;
    k_fmu = 1;
    k_ctl = 1;

    % Time search variables
    dt_fmu = model.dt_fmu;
    t_ctl = 0;
    t_fmu = 0;
    
    % Carrier variables
    x_fmu = x_bar(:,1);
    u_prev  = zeros(5,1);
    u_curr  = u_bar(:,1);    
    ld_curr = al.lambda(:,1);
    mu_curr = al.mu(:,1);
    
    % Cost variables
    J = 0;
    c_con = zeros(al.n_con,N);

    % Actual forward pass
    while k_ctl < N
        % Calculate current time
        t_now = (k_fmu-1) * dt_fmu;
        
        % Check if we have reached control time marker
        if t_now >= t_ctl
            % Assign to nominal state
            if init_flag == 1
                del_u = zeros(5,1);
            else
                del_x = x_fmu - x_bar(:,k_ctl);
                del_u = model.alpha.*(l(:,:,k_ctl) + L(:,:,k_ctl)*del_x);
            end
            u_curr = u_curr + del_u;

            x_bar(:,k_ctl) = x_fmu;
            u_bar(:,k_ctl) = u_curr;
            % Compute stagewise cost
            err_x = x_bar(:,end) - x_star;
            J_lqr = 0.5* err_x'*Q_t*err_x + 0.5*u_bar(:,k_ctl)'*R*u_bar(:,k_ctl);
            
            [I_mu,c_con_k,~,~] = con_builder(u_curr,u_prev,ld_curr,mu_curr,model);
            J_al = (ld_curr + 0.5.*I_mu*c_con_k)'*c_con_k;
            J = J + J_lqr + J_al;
            
            c_con(:,k_ctl) = c_con_k;

            % Update motor command
            m_cmd = wrench2m_controller(u_bar(1:4,k_ctl),model);
            
            % Update control time marker
            t_step = (u_bar(5,k_ctl))^2;
            t_ctl = t_ctl + t_step;

            % Prep for next marker
            k_ctl = k_ctl + 1;
            u_prev = u_curr;
            if k_ctl < N
                u_curr = u_bar(:,k_ctl);
                ld_curr = al.lambda(:,k_ctl);
                mu_curr = al.mu(:,k_ctl);
            end
        end

        % Update dynamics according to fmu time
        x_fmu = quadcopter(x_fmu,m_cmd,model,FT_ext,'fmu');
        
        % Update fmu counter
        k_fmu = k_fmu + 1;
    end

    % Terminal Case
    x_bar(:,k_ctl) = x_fmu;

    err_x = x_bar(:,end) - x_star;
    J_lqr = 0.5* err_x'*Q_f*err_x;
    J = J + J_lqr;
end