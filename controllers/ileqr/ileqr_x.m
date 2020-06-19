function [nom, R_gamma] = ileqr_x(n,x_now,wp,nom,wts,model)
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

    W_inv = model.W_leqr_inv;
    gamma = model.gamma;
    
    % Convergence Variables
    itrs = 1;
    x_diff = 1000;
    while x_diff > 1e-1
        % Initialize x_bar variable
        x_itr = x_bar;
        x_itr(:,end) = x_feed;
        u_itr = u_bar;

        % Determine A and B matrices for this step
        [A,B] = dynamics_linearizer(x_bar,u_bar,model);
        
        % Backward Pass   
        [l,L] = ileqr_bp(x_itr,x_bar,u_bar,A,B,Q_t,Q_f,R,W_inv,gamma);
        
        % Forward Pass
        [x_bar,u_bar,R_gamma] = ileqr_fp(x_bar,u_bar,x_now,l,L,model,Q_t,Q_f,R);
        
        % Check for Convergence
        if itrs < 100
            x_diff = sum(vecnorm(x_bar-x_itr))/(N-n);
            u_diff = sum(vecnorm(u_bar-u_itr))/(N-n);
%             disp(['[ilqr]: Iteration ',num2str(itrs),'  del_x difference: ',num2str(x_diff),'  del_u difference: ',num2str(u_diff)]);

            itrs = itrs + 1;
        else
%             x_bar = nom.x_bar(:,n:end);
%             u_bar = nom.u_bar(:,n:end);
            disp('[ileqr_x]: Convergence Timeout. Using last compute (TODO: Switch a line-search method).');
            break;
        end
    end
    
    % Update the Nominal Values
    nom.x_bar(:,n:N) = x_bar;
    nom.u_bar(:,n:N-1) = u_bar;
    nom.l(:,:,n:N-1) = l;
    nom.L(:,:,n:N-1) = L;
     
%     comp_pcnt = 100*toc/model.dt_ctl;
%     disp(['[ileqr_x]: iLQR Converged on Iteration ',num2str(itrs),' in ',num2str(toc),' seconds using ',num2str(comp_pcnt), '% of available time']);
end

