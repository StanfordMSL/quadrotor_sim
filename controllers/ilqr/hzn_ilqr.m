function [nom, fs_trig] = hzn_ilqr(x_now,wp,nom,fc,model,t_hzn)
    t_solve = tic;
    fs_trig = 0;
    
    % Determine where we are along the nominal trajectory.
    n = model.con_hz*model.lqr_dt;
    N = nom.total;

    % Unpack the Terms
    t_bar = linspace(0,t_hzn,N);
    x_bar = [nom.x_bar(:,n:end) repmat(nom.x_bar(:,end),1,(n-1))];
    u_bar = [nom.u_bar(:,n:end) repmat(nom.u_bar(:,end),1,(n-1))];  
%     u_bar = [nom.u_bar(:,n:end) zeros(4,(n-1))];  

    % Convergence Variables
    itrs = 1;
    x_diff = 999;
    while x_diff > 1e-3
        % Determine A and B matrices for this step
        [A,B] = dynamics_linearizer(x_bar,u_bar,model);
        
        % Backward Pass   
        [l,L] = ilqr_bp(t_bar,x_bar,u_bar,wp,A,B,N,fc);
        
        % Forward Pass
        [x_bar,u_bar,~] = ilqr_fp(t_bar,x_bar,u_bar,x_now,wp,l,L,N,nom.alpha,model,fc);
        
        % Detect Singularities
        [~,msgid] = lastwarn;
        if strcmp(msgid,'MATLAB:nearlySingularMatrix') 
            fs_trig = 1;
            disp('[hzn_ilqr]: Near singularity detected. Commands cut)');
            break;
        end
        
        % Check for Convergence
        if itrs < 100
            x_diff = norm(x_bar(:,end) - wp.x(:,end));

            itrs = itrs + 1;
        else
            disp('[hzn_ilqr]: Convergence Timeout. Using last compute (TODO: Switch a line-search method).');
            break;
        end
    end
    
    % Update the Nominal Values
    nom.x_bar = x_bar;
    nom.u_bar = u_bar;
    nom.l = l;
    nom.L = L;
    
    disp(['[ilqr]: iLQR Compute Exited on Iteration ',num2str(itrs),' and taking ',num2str(toc(t_solve)),' seconds.']);
end

