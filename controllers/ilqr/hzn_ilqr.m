function nom = hzn_ilqr(x_now,wp,nom,fc,model,t_hzn)
    tic
    % Determine where we are along the nominal trajectory.
    [~,n] = min(vecnorm(nom.x_bar - x_now));
    N = nom.total;

    % Unpack the Terms
    t_bar = linspace(0,t_hzn,N);
    x_bar = [nom.x_bar(:,n:end) repmat(nom.x_bar(:,end),1,(n-1))];
    u_bar = [nom.u_bar(:,n:end) repmat(nom.u_bar(:,end),1,(n-1))];  
    
    % Convergence Variables
    itrs = 1;
    u_diff_avg = 10;
    while u_diff_avg > 1e-1
        % Determine A and B matrices for this step
        [A,B] = dynamics_linearizer(x_bar,u_bar,model);
        
        % Backward Pass   
        [l,L] = ilqr_bp(t_bar,x_bar,u_bar,wp,A,B,N,fc);
        
        % Forward Pass
        [x_bar,u_bar,u_diff] = ilqr_fp(t_bar,x_bar,u_bar,x_now,wp,l,L,N,nom.alpha,model,fc);

        % Check for Convergence
        if itrs < 100
            u_diff_avg = u_diff/N;
%             disp(['[ilqr]: Iteration ',num2str(itrs),'  del_u difference: ',num2str(u_diff_avg)]);

            itrs = itrs + 1;
        else
%             x_bar = nom.x_bar(:,n:end);
%             u_bar = nom.u_bar(:,n:end);
            disp('[ilqr]: Convergence Timeout. Using last compute (TODO: Switch a line-search method).');
            break;
        end
    end
    
    % Update the Nominal Values
    nom.x_bar = x_bar;
    nom.u_bar = u_bar;
    nom.l = l;
    nom.L = L;
    
    disp(['[ilqr]: iLQR Compute Successful on Iteration ',num2str(itrs),' and taking ',num2str(toc),' seconds.']);
end

