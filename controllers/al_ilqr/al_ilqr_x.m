function [nom, J] = al_ilqr_x(n,x_now,wp,nom,wts,model)
    tic
    % Determine current point along trajectory and remainder of points
    idx_N = find(nom.wp_fr > n,1);
    N = floor(nom.wp_fr(idx_N));
    
    % Unpack the Terms
    x_bar = nom.x_bar(:,n:N);
    u_bar = nom.u_bar(:,n:N-1); 
    
    idx = wp.Q_key(idx_N-1,1);
    Q_t = wts.Q(:,:,idx);
    idx = wp.Q_key(idx_N-1,2);
    Q_f = wts.Q(:,:,idx);
    
    R = wts.R;
    
    x_feed = wp.x(:,idx_N);
    % Convergence Variables
    itrs = 1;
    x_diff = 1000;
    
    % Initialize AL variable %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    N_al = N-n;
    A_cons = (1/model.kt_act(1,1)).*model.m2w_inv;
    b_cons = model.motor_min.^2 * ones(4,1);
    mu = ones(4,N_al);
    lambda = 0.*ones(4,N_al);
    phi = 10;
    c_cons = zeros(4,N_al);
    max_c_cons_diff = 9999;
    max_c_cons = 9999;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    while max_c_cons_diff > 1
        % Run the AL_ilqr %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        while x_diff > 1e-3
            % Initialize x_bar variable
            x_itr = x_bar;
            x_itr(:,end) = x_feed;

            % Determine A and B matrices for this step
            [A,B] = dynamics_linearizer(x_bar,u_bar,model);

            % Backward Pass   
            [l,L] = al_ilqr_bp(x_itr,x_bar,u_bar,A,B,Q_t,Q_f,R,A_cons,b_cons,mu,lambda);

            % Forward Pass
            [x_bar,u_bar,J] = al_ilqr_fp(x_bar,u_bar,x_now,l,L,nom.alpha,model,Q_t,Q_f,R);

%             motor_debug(x_bar,u_bar,model);
            % Check for Convergence
            if itrs < 100
                x_diff = sum(vecnorm(x_bar-x_itr))/(N-n);
                itrs = itrs + 1;
            else
                disp('[al_ilqr_x]: Convergence Timeout. Using last compute (TODO: Switch a line-search method).');
                break;
            end
        end
        % Update lambda and mu and c_con %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        for k = 1:N_al
            lambda_cand = lambda(:,k) + mu(:,k).*(A_cons*u_bar(:,k)-b_cons);
            
            lambda(:,k) = max(zeros(4,1),lambda_cand);
            mu(:,k) = phi.*mu(:,k);
            c_cons(:,k)   = -A_cons*u_bar(:,k) + b_cons ;
        end
        max_c_cons_old = max_c_cons;
        max_c_cons = max((max(c_cons)));
        disp(['[al_ilqr_x]: Max(c): ',num2str(max_c_cons)]);
        max_c_cons_diff = abs(max_c_cons - max_c_cons_old);
    end
    
    % Update the Nominal Values
    nom.x_bar(:,n:N) = x_bar;
    nom.u_bar(:,n:N-1) = u_bar;
    nom.l(:,:,n:N-1) = l;
    nom.L(:,:,n:N-1) = L;
    
%     comp_pcnt = 100*toc/model.dt_ctl;
%     disp(['[ilqr_x]: iLQR Converged on Iteration ',num2str(itrs),' in ',num2str(toc),' seconds using ',num2str(comp_pcnt), '% of available time']);
end

