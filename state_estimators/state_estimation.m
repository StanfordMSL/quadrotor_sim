function ses = state_estimation(x_now,u_now,ses,mode)
    switch mode
        case 'bypass'
            x = x_now;
            sigma = zeros(13,13);
        case 'normal'
            % Unpack some stuff
            dt = model.dt_fmu;
            C = model.C;
            Q = model.Q_fil;
            R = model.R_fil;

            % Generate Noise
            vt = R*randn(10,1);

            y = C*x_now + vt;

            % Predict Forward Dynamics
            FT_ext = zeros(6,1);
            wt = zeros(13,1);
            x_pred = quadcopter_est(x_now,u_now,FT_ext,wt);

            % Predict the Covariance
            A = A_ekf_calc(x_now,u_mt);      
            sig_pred = A*ses.sigma*A'+Q;

            % Update 
            g_ekf = sensors(x_pred,model);

            K = sig_pred*C'/(C*sig_pred*C'+R);

            x   = x_pred  + K*(y-g_ekf);
            x(7:10,1) = x(7:10,1)./norm(x(7:10,1));
            
            sigma = sig_pred - K*C*sig_pred;
    end
    
    ses.x = x;
    ses.u = u_now;
    ses.sigma = sigma;
end