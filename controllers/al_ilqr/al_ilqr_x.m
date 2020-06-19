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
    
    % Target state
    x_star = wp.x(:,idx_N);
    
    % Candidate Holder
    x_cand = x_bar;
    u_cand = u_bar;
    J_cand = 0;
    
    % Initialize AL Variables %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    N_al = N-n;
    n_con = 12;
    
    mu = ones(n_con,N_al);
    lambda = 0.*ones(n_con,N_al);
    phi = 10;
    c_cons = zeros(n_con,N_al);
    con_check = 999;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    while con_check > 0
        % Convergence Variables
        itrs = 1;
        x_diff = 1000;
        % Run the AL_ilqr %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        while x_diff > 1e-1
            % Initialize x_bar variable
            x_itr = x_bar;
            x_itr(:,end) = x_star;
        
            % Determine A and B matrices for this step
            [A,B] = dynamics_linearizer(x_bar,u_bar,model);

            % Backward Pass   
            [l,L,c_cons] = al_ilqr_bp(x_itr,x_bar,u_bar,A,B,Q_t,Q_f,R,mu,lambda,model,wp,n_con);

            % Forward Pass
            [x_bar,u_bar,J] = al_ilqr_fp(x_bar,u_bar,x_now,l,L,model,Q_t,Q_f,R);
%             direct_plot(x_bar,wp,'debug','show');

%             motor_debug(x_bar,u_bar,model);
            % Check for Convergence
            if itrs <= 10
                x_diff_old = x_diff;
                x_diff = norm(x_bar(:,end) - x_star);
            
                if x_diff < x_diff_old
                    x_cand = x_bar;
                    u_cand = u_bar;
                    J_cand = J;
%                     disp('[ilqr]: Candidate Improved');
                end
%                 disp(['[al_ilqr_x]: Iteration ',num2str(itrs),'  del_x difference: ',num2str(x_diff)]);

                itrs = itrs + 1;  
            else
                % Out of Time. Take Your Best Guess
                x_bar = x_cand;
                u_bar = u_cand;
                J = J_cand;
            
%                 disp('[al_ilqr_x]: Convergence Timeout. Using best candidate).');
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
    nom.x_bar(:,n:N) = x_bar;
    nom.u_bar(:,n:N-1) = u_bar;
    nom.l(:,:,n:N-1) = l;
    nom.L(:,:,n:N-1) = L;
end

