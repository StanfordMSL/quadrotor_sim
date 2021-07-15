function ses = state_estimation(x_now,ses,mode)
    switch mode
        case 'bypass'
            x = x_now;
            sigma = zeros(13,13);
        case 'ekf'
            % Unpack some stuff
            C = ses.C;
            Q = ses.Q;
            R = ses.R;

            % Generate Noise
            vt = R*randn(10,1);
            y = C*x_now + vt;

            % Predict Forward Dynamics
            FT_ext = zeros(6,1);
            wt = zeros(13,1);
            x_pred = quadcopter_est(ses.x,ses.u,FT_ext,wt);

            % Predict the Covariance
            A = A_ekf_calc(ses.x,ses.u);      
            sig_pred = A*ses.sigma*A'+Q;

            % Update 
            g_ekf = C*x_pred;

            K = sig_pred*C'/(C*sig_pred*C'+R);

            x = x_pred + K*(y-g_ekf);
            x(7:10,1) = x(7:10,1)./norm(x(7:10,1));
            
            sigma = sig_pred - K*C*sig_pred;
    end
    
    ses.x = x;
    ses.sigma = sigma;
end