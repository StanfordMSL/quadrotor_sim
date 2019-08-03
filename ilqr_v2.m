function [m_cmd_curr,fc] = ilqr_v2(x_fc,model,fc,kf_t)
    % Some Initialization Stuff
    fc.frame = fc.frame + 1;
    
    if fc.frame == 1
        fc = ilqr_init(fc,kf_t,model);
    end
    
    index = fc.frame;
    N = fc.fr_total;
    
    % Unpack the Terms
    Q = fc.Q;
    R = fc.R;
    x_bar = zeros(12,2);
    u_bar = zeros(4,1);
    % Initialize the iLQR variables
    m_cmd = zeros(4,N);

    % Convergence variables
    itrs = 0;
    conv_crit = 10;

    while conv_crit > 1e-9
        itrs = itrs + 1;

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Forward Pass
        FT_ext = zeros(6,1);

        m_cmd = wrench2m_cmd(u_bar,model);
        x_bar(:,2) = quadcopter(x_bar(:,1),m_cmd,model,FT_ext,'fc');
        
        A = A_calc_wrench(x_bar(:,1),u_bar,model);
        B = B_calc_wrench(x_bar(:,1),model);
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Backward Pass 
        del_x = x_fc - x_bar(:,1);
        x_N = kf_t.x(:,2);
        u_bar = -inv(R+B'*Q*B)*(B'*Q*(x_bar(:,end)+A*del_x-x_N-B*u_bar));
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Check for Convergence
        conv_crit = norm(x_N - x_bar(:,end));
        disp(x_bar);
    end

    fc.x_bar(:,index:end) = x_bar;
    fc.u_bar(:,index:end) = u_bar;

    % Compute Motor Commands
    m_cmd_curr = m_cmd(:,1);

    for k = 1:4
        if m_cmd(k,1) < model.motor_min
            m_cmd(k,1) = model.motor_min;
            disp('[ilqr]: Exceeded Motor Min');
        elseif m_cmd(k,1) > model.motor_max
            m_cmd(k,1) = model.motor_max;
            disp('[ilqr]: Exceeded Motor Max');
        end
    end
end

