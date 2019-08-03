function [mu_new, sigma_new] = iukf(mu_curr,sigma_curr,u,y,time,model,x_act,quat)
    lambda = 2;
    dim = size(mu_curr,1);
    total_sp = (2*dim+1);
    
    % Unscented Transform 
    sig_points = UT(mu_curr,sigma_curr,dim,lambda);
    % Predict %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    sigp_pred = zeros(dim,total_sp);
    for j = 1:total_sp
        sigp_pred(1:3,j) = sig_points(1:3,j) + time.fc_dt*mu_curr(4:6,1);

        % Predict Velocities
        vel_dot = lin_acc(sig_points(:,j),u,model,[0 0 0]',0,1);
        sigp_pred(4:6,j) = sig_points(4:6,j) + time.fc_dt*vel_dot;

        % Predict Quaternions
        % omegas
        wx = sig_points(11,j);
        wy = sig_points(12,j);
        wz = sig_points(13,j);

        % Setup Some Useful Stuff for Pred %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        Omega = [ 0 -wx -wy -wz ;...
                 wx   0  wz -wy ;...
                 wy -wz   0  wx ;...
                 wz  wy -wx   0 ];

        q_hat = sig_points(7:10,j) + 0.5*Omega*sig_points(7:10,j)*time.fc_dt;
        sigp_pred(7:10,j) = q_hat./norm(q_hat);

        % Predict Angular Velocities
        omega_dot = ang_acc(u,sig_points(11:13,j),model,[0 0 0]',1);
        sigp_pred(11:13,j) = sig_points(11:13,j) + omega_dot*time.fc_dt;
    end
     
    actual = [x_act(1:6,1) ; quat ; x_act(10:12,1)]
    [mu_ukf_pred, sigma_ukf_pred_bar] = invUT(sigp_pred,dim,lambda);
    mu_ukf_pred(7:10,1) = mu_ukf_pred(7:10,1)./norm(mu_ukf_pred(7:10,1));
    sigma_ukf_pred = sigma_ukf_pred_bar + model.Q_fil;
    
    % Update
    sigp_upd = UT(mu_curr,sigma_curr,dim,lambda);
    
    sigp_g = zeros(13,(2*dim+1));  
    for j = 1:total_sp
        sigp_g(:,j) = sensors(time,sigp_upd(:,j),u,model,1);
    end
    
    [mu_y, sigma_y] =invUT(sigp_g,dim,lambda);
    
    sigma_y = sigma_y + model.R_fil;
    sigma_xy = cross_cov(mu_ukf_pred,sigp_pred,mu_y,sigp_g,dim,lambda);
    
    limit = 1000;
    mu_itr = zeros(13,limit);
    sigma_itr = zeros(13,13,limit);
    
    mu_itr(:,1) = mu_ukf_pred;
    mu_itr(7:10,1) = mu_itr(7:10,1)/norm(mu_itr(7:10,1));
    mu_itr(:,2) = mu_ukf_pred + sigma_xy*inv(sigma_y)*(y-mu_y);
    mu_itr(7:10,2) = mu_itr(7:10,2)/norm(mu_itr(7:10,2));
    sigma_itr(:,:,1) = sigma_ukf_pred;
    sigma_itr(:,:,2) = sigma_ukf_pred - sigma_xy*inv(sigma_y)*sigma_xy';
    
    % And So We Begin Iterating!!! %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    g = 1;
    
    for j = 2:limit
        sig_points_itr = UT(mu_itr(:,j-1),sigma_itr(:,:,j-1),dim,lambda);

        [mu_itr_pred,sigma_itr_pred] = invUT(sig_points_itr,dim,lambda);
        mu_itr_pred(7:10,1) = mu_itr_pred(7:10,1)./norm(mu_itr_pred(7:10,1));
        
        sigp_g_itr = zeros(13,(2*dim+1));  
        for k = 1:total_sp
            sigp_g_itr(:,k) = sensors(time,sig_points_itr(:,k),u,model,1);
        end
        
        [mu_y_itr, sigma_y_itr] =invUT(sigp_g_itr,dim,lambda);
        
        sigma_y_itr = sigma_y_itr + model.R_fil;
        sigma_xy_itr = cross_cov(mu_itr_pred,sig_points_itr,mu_y_itr,sigp_g_itr,dim,lambda);
        
        K_itr = sigma_xy_itr*inv(sigma_y_itr);
        
        mu_itr(:,j) = mu_itr_pred + g*K_itr*(y-mu_y_itr);
        mu_itr(7:10,j) = mu_itr(7:10,j)/norm(mu_itr(7:10,j));
        sigma_itr(:,:,j) = sigma_itr_pred-K_itr*sigma_y_itr*K_itr';
                
        % check the iteration
        y_tilde = sensors(time,mu_itr(:,j),u,model,1);
        y_tilde_old = sensors(time,mu_itr(:,j-1),u,model,1);

        x_bar = mu_itr(:,j) - mu_itr(:,j-1);
        y_bar = y-y_tilde;
        
        inv_R = inv(model.R_fil);
        J =  x_bar'*inv(sigma_itr(:,:,j-1))*x_bar + y_bar'*inv_R*y_bar;
        threshold = y_tilde_old'*inv_R*y_tilde_old;
        
        if J < threshold
            g = 0.1*g;
        else
            disp('Uh oh');
            break
        end
    end
    mu_new = mu_itr(:,j);
    mu_new(7:10,1) = mu_new(7:10,1)/norm(mu_new(7:10,1));
    sigma_new = sigma_itr(:,:,j);
end