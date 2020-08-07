function nom = toc_ilqr(x_star,nom,wts,wp,model,reps)
    tic
        
    % Convergence Variables
    itrs = 1;
    J_curr_best = 1e9;
    J_N_best = 1e9;

    J_tol  = 1e-3;         % temporary sophistication. to be given more thought.
    % Run the toc-ilqr %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    while J_N_best > J_tol            
        % Forward Pass
        nom = toc_ilqr_fp(nom,model);
        [J_curr,J_N_curr,c_aug] = toc_J_calc(x_star,nom,wts);

%         if nom.N_loop > 1
%             fast_animation_plot(nom.x_bar,wp,'persp')
%         end
        
        % Determine A and B matrices for this step
        [A,B] = dynamics_linearizer(nom.x_bar,nom.u_bar,model);

        % Backward Pass         
        nom = toc_ilqr_bp(x_star,nom,A,B,wts,model);
        % Check for improvement
        if J_N_curr <= J_N_best 
            nom_cand = nom;
            nom_cand.J     = J_curr;
            nom_cand.J_N   = J_N_curr;
            nom_cand.c_aug = c_aug;
            J_curr_best = J_curr;
            J_N_best = J_N_curr;
        end

        if (itrs <= reps) && (J_curr < (10*J_curr_best))
            itrs = itrs + 1;
        else
            nom = nom_cand;
            break
        end
    end
    
    disp(['[toc_ilqr]: Total Cost: ',num2str(J_curr_best),'  || End Cost: ',num2str(J_N_best)]);

end

