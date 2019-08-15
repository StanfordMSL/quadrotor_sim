function nom = ilqr(t_now,x_now,wp,nom,fc,model)
    tic
    % Determine current point along trajectory and remainder of points
    n = find(nom.t_bar > t_now,1)-1;
    N = nom.total-n+1;
    
    % Unpack the Terms
    t_bar = nom.t_bar(:,n:end);
    x_bar = nom.x_bar(:,n:end);
    u_bar = nom.u_bar(:,n:end);  
    
    % Convergence Variables
    itrs = 1;
    u_diff_avg = 10;
    while u_diff_avg > 1e-1
        % Determine A and B matrices for this step
        [A,B] = dynamics_linearizer(x_bar,u_bar,model);
        
        % Backward Pass   
        [l,L] = ilqr_bp(t_bar,x_bar,u_bar,wp,A,B,N,fc);
        
        % Forward Pass
        [x_bar,u_bar,u_diff] = ilqr_fp(t_bar,x_bar,u_bar,x_now,wp,l,L,N,model,fc);

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
    nom.x_bar(:,n:end) = x_bar;
    nom.u_bar(:,n:end) = u_bar;
    nom.l(:,:,n:end) = l;
    nom.L(:,:,n:end) = L;
    
    disp(['[ilqr]: iLQR Compute Successful on Iteration ',num2str(itrs),' and taking ',num2str(toc),' seconds.']);
end

