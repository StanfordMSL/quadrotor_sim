function [nom, fs_trig] = hzn_ilqr(x_now,wp,nom,wts,model)
    tic
    fs_trig = 0;
    
    % Determine where we are along the nominal trajectory.
    n = model.fbc_hz*model.lqr_dt;
    N = nom.total;

    % Unpack the Terms and Inject Weights
    x_bar = [nom.x_bar(:,n:end) repmat(nom.x_bar(:,end),1,(n-1))];
    u_bar = [nom.u_bar(:,n:end) repmat(nom.u_bar(:,end),1,(n-1))];  

    Q_t = wts.Q(:,:,6);
    Q_f = wts.Q(:,:,6);
    
    R = wts.R;
    
    x_feed = wp.x(:,2);
    % Convergence Variables
    itrs = 1;
    x_diff = 1000;
    while x_diff > 1e-1
        % Initialize x_bar variable
        x_itr = x_bar;
        x_itr(:,end-9:end) = repmat(x_feed,1,10);
%         u_itr = u_bar;

        % Determine A and B matrices for this step
        [A,B] = dynamics_linearizer(x_bar,u_bar,model);
        
        % Backward Pass   
        [l,L] = ilqr_bp(x_itr,x_bar,u_bar,A,B,Q_t,Q_f,R);
        
        % Forward Pass
        [x_bar,u_bar] = ilqr_fp(x_bar,u_bar,x_now,l,L,model,Q_t,Q_f,R);
        
        % Detect Singularities
        [~,msgid] = lastwarn;
        if strcmp(msgid,'MATLAB:nearlySingularMatrix') 
            fs_trig = 1;
            disp('[hzn_ilqr]: Near singularity detected. Commands cut)');
            break;
        end       
        
        % Check for Convergence
        if itrs < 100
            x_diff = sum(vecnorm(x_bar-x_itr))/(N-n);
%            u_diff = sum(vecnorm(u_bar-u_itr))/(N-n);
%            disp(['[ilqr]: Iteration ',num2str(itrs),'  del_x difference: ',num2str(x_diff),'  del_u difference: ',num2str(u_diff)]);

            itrs = itrs + 1;
        else
%             x_bar = nom.x_bar(:,n:end);
%             u_bar = nom.u_bar(:,n:end);
            disp('[hzn_ilqr]: Convergence Timeout. Using last compute (TODO: Switch a line-search method).');
            break;
        end
    end
    
    % Update the Nominal Values
    nom.x_bar = x_bar;
    nom.u_bar = u_bar;
    nom.l = l;
    nom.L = L;
    
    comp_pcnt = 100*toc/model.ctl_dt;
    disp(['[hzn_ilqr]: iLQR Converged on Iteration ',num2str(itrs),' in ',num2str(toc),' seconds using ',num2str(comp_pcnt), '% of available time']);
end


