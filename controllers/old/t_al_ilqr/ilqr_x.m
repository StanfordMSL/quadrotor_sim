function [nom, J] = ilqr_x(n,x_now,wp,nom,wts,model)
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
    
    % Candidate Holder
    x_cand = x_bar;
    u_cand = u_bar;
    
    while x_diff > 1e-1
        % Initialize x_bar variable
        x_itr = x_bar;
        x_itr(:,end) = x_feed;

        % Determine A and B matrices for this step
        [A,B] = dynamics_linearizer(x_bar,u_bar,model);
        
        % Backward Pass   
        [l,L] = ilqr_bp(x_itr,x_bar,u_bar,A,B,Q_t,Q_f,R);
        
        % Forward Pass
        [x_bar,u_bar,J] = ilqr_fp(x_bar,u_bar,x_now,l,L,model,Q_t,Q_f,R);

        % Check for Convergence
        if itrs <= 10
            x_diff_old = x_diff;
            x_diff = sum(vecnorm(x_bar-x_itr))/(N-n);
            
            if x_diff < x_diff_old
                x_cand = x_bar;
                u_cand = u_bar;
%                 disp('[ilqr]: Candidate Improved');
            end
%             disp(['[ilqr]: Iteration ',num2str(itrs),'  del_x difference: ',num2str(x_diff)]);

            itrs = itrs + 1;
        else
            x_bar = x_cand;
            u_bar = u_cand;
%             disp('[ilqr_x]: Convergence Timeout. Using best candidate).');
            break;
        end
    end
    
    % Update the Nominal Values
    nom.x_bar(:,n:N) = x_bar;
    nom.u_bar(:,n:N-1) = u_bar;
    nom.l(:,:,n:N-1) = l;
    nom.L(:,:,n:N-1) = L;
    
%     comp_pcnt = 100*toc/model.dt_ctl;
%     disp(['[ilqr_x]: iLQR Converged on Iteration ',num2str(itrs),' in ',num2str(toc),' seconds using ',num2str(comp_pcnt), '% of available time']);
end

