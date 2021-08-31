function ses = state_estimation(x_now,x_bar,ses,mode)
    
    % Integral Dynamics
    p_bar = x_bar(1:3,1);
    q_bar = x_bar(7:10,1);
    z = error_upd(ses.z,x_now,p_bar,q_bar);

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
    
    s = [x(1:10) ; z];

    ses.x = x;
    ses.z = z;
    ses.s = s;
    ses.sigma = sigma;
end