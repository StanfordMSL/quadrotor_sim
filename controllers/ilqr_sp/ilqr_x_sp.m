function nom = ilqr_x_sp(x_curr,x_wp,nom,wts,model)
    tic
        
    % Unpack some stuff
    x_bar = nom.x_bar;
    u_bar = nom.u_bar;
    x_star = [x_wp ; model.dt_ctl_min];
    
    % Candidate Holder
    x_cand = x_bar;
    u_cand = u_bar;
    J_cand = 0;
    
    % Initialize AL Variables %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    N_al = size(x_bar,2);
    n_con = 2;
    
    mu = ones(n_con,N_al);
    lambda = 0.*ones(n_con,N_al);
    phi = 10;
    c_cons = zeros(n_con,N_al);
    con_check = 999;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    while con_check > 0
        % Convergence Variables
        itrs = 1;
        x_diff = 1e9;
        % Run the AL_ilqr %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        while x_diff > 1e-1
            % Determine A and B matrices for this step
            [A,B] = dynamics_linearizer(x_bar,u_bar,model);
        
            % Backward Pass   
            [l,L,c_cons] = ilqr_bp_sp(x_star,x_bar,u_bar,A,B,wts,mu,lambda,model,n_con);
        
            % Forward Pass
            [x_bar,u_bar,J] = ilqr_fp_sp(x_curr,x_star,x_bar,u_bar,l,L,wts,model);

            % Check for Convergence
            if itrs <= 10
                x_diff_old = x_diff;
                x_diff = J;
            
                if x_diff < x_diff_old
                    x_cand = x_bar;
                    u_cand = u_bar;
                    J_cand = J;
    %                   disp('[ilqr]: Candidate Improved');
                end
    %             disp(['[ilqr]: Iteration ',num2str(itrs),'  del_x difference: ',num2str(x_diff)]);

                itrs = itrs + 1;
            else
                x_bar = x_cand;
                u_bar = u_cand;
                J = J_cand;
%               disp('[ilqr_x]: Convergence Timeout. Using best candidate).');
                break;
            end
        end
        % Update lambda and mu and c_con %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        for k = 1:N_al
            lambda_cand = lambda(:,k) + mu(:,k).*c_cons(:,k);
            
            lambda(:,k) = max(zeros(n_con,1),lambda_cand);
            mu(:,k) = phi.*mu(:,k);
        end
        con_check = sum(any(c_cons > 1e-6));
        disp(['[al_ilqr_x]: Unfulfilled constraints: ',num2str(con_check)]);
    end
    
    % Update the Nominal Values
    nom.x_bar = x_bar;
    nom.u_bar = u_bar;
    nom.l = l;
    nom.L = L;
    
%     comp_pcnt = 100*toc/model.dt_ctl;
%     disp(['[ilqr_x]: iLQR Converged on Iteration ',num2str(itrs),' in ',num2str(toc),' seconds using ',num2str(comp_pcnt), '% of available time']);
end

