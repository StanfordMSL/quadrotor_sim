function nom = ilqr_x_sp(x_curr,x_star,nom,wts,wp,model)
    tic
        
    % Unpack some stuff
    x_bar  = nom.x_bar;
    u_bar  = nom.u_bar;
    l      = nom.l;
    L      = nom.L;
    
    % Candidate
    x_cand = x_bar;
    u_cand = u_bar;
    l_cand = l;
    L_cand = L;
    
    % Initialize Augmented Lagrangian variables
    al = al_init(3,model);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    con_check = 999;
    while con_check > 0
        % Convergence Variables
        itrs = 1;
        J_curr = 1e9;
        J_prev = 1e9;
        J_tol  = 1e-9.*sum(diag(wts.Q_pstn))*model.N_ctl;         % temporary sophistication. to be given more thought.
        % Run the al-ilqr %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        while J_curr > J_tol            
            % Forward Pass
            if itrs == 1
                [x_bar,u_bar,J_curr,c_con] = ilqr_fp_sp(x_bar,u_bar,l,L,x_star,al,wts,model,1);
            else
                [x_bar,u_bar,J_curr,c_con] = ilqr_fp_sp(x_bar,u_bar,l,L,x_star,al,wts,model,0);
            end
            fast_animation_plot(x_bar,wp,'persp')
            
            % Determine A and B matrices for this step
            [A,B] = dynamics_linearizer(x_bar,u_bar,model);
        
            % Backward Pass   
            [l,L] = ilqr_bp_sp(x_star,x_bar,u_bar,A,B,wts,model,al);
            
            % Check for improvement
            if J_curr < J_prev
                x_cand = x_bar;
                u_cand = u_bar;
                l_cand = l;
                L_cand = L;

                J_prev = J_curr;
                disp(['[ilq_x_sp]: Current Cost: ',num2str(J_curr)]);

            end
            
            if itrs <= 10
                itrs = itrs + 1;
            else
                % Out of Time. Take Your Best Guess
                x_bar = x_cand;
                u_bar = u_cand;
                l     = l_cand;
                L     = L_cand;
                
                break
            end
        end
        % Update lambda and mu and c_con %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        for k = 1:al.N_al
            lambda_cand = al.lambda(:,k) + al.mu(:,k).*c_con(:,k);
            
            al.lambda(1:2,k) = max(zeros(2,1),lambda_cand(1:2,1));
            al.lambda(3,k)   = lambda_cand(3,1);
            
            al.mu(:,k) = al.phi.*al.mu(:,k);
        end
        con_check = sum(any(c_con > 1e-6));
        disp(['[al_ilqr_x]: Unfulfilled constraints: ',num2str(con_check)]);
        con_check = -1;
    end
    
    % Update the Nominal Values
    nom.x_bar = x_cand;
    nom.u_bar = u_cand;
    nom.l = l_cand;
    nom.L = L_cand;
    
%     comp_pcnt = 100*toc/model.dt_ctl;
%     disp(['[ilqr_x]: iLQR Converged on Iteration ',num2str(itrs),' in ',num2str(toc),' seconds using ',num2str(comp_pcnt), '% of available time']);
end

