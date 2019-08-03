function [m_cmd,fc] = ilqr(x_fc,model,fc,kf_t)
    % Check for initial and determine number of steps to compute.
    index = fc.frame;

    if index == 1
        fc = ilqr_init(fc,kf_t,model,x_fc);
    elseif (index == fc.fr_total) && (fc.wp < kf_t.N_wp)
        fc.frame = 0;
    elseif (index >= fc.fr_total) && (fc.wp == kf_t.N_wp)
        index = fc.fr_total;
    end
    fc.frame = fc.frame + 1;

    x_wp = kf_t.x(:,fc.wp);
    N = fc.fr_total-index+1;

    % Unpack the Terms    
    x_bar = fc.x_bar(:,index:end);
    u_bar = fc.u_bar(:,index:end);   

    % Initialize the iLQR variables
    l = zeros(4,1,N);
    L = zeros(4,12,N);

    % Convergence variables
    itrs = 0;
    del_u_tol = 10;

    while del_u_tol > 1e-1
        itrs = itrs + 1;

        % Forward Pass
        [x_bar,u_bar,del_u,A,B] = ilqr_fp(x_bar,u_bar,x_fc,l,L,N,model,fc);

        % Backward Pass   
        [l,L] = ilqr_bp(x_bar,u_bar,x_wp,A,B,N,fc);

        % Check for Convergence
        if itrs == 1
            % Do nothing
        elseif itrs > 1 && itrs < 1000
            del_u_tol = sum(vecnorm(del_u))/N;
        else
            x_bar = fc.x_bar(:,index:end);
            u_bar = fc.u_bar(:,index:end);
            disp('[ilqr]: convergence timeout');
            break;
        end
    end

    fc.x_bar(:,index:end) = x_bar;
    fc.u_bar(:,index:end) = u_bar;

    % Compute Motor Commands
    m_cmd = wrench2m_cmd(u_bar(:,1),model);

    for k = 1:4
        if m_cmd(k,1) < model.motor_min
            m_cmd(k,1) = model.motor_min;
            
            if m_cmd(k,1) <  (model.motor_min - 1e-6)
                disp('[ilqr]: Exceeded Motor Min');
            end
            
        elseif m_cmd(k,1) > model.motor_max
            m_cmd(k,1) = model.motor_max;
            
            if m_cmd(k,1) > (model.motor_max + 1e-6)
                disp('[ilqr]: Exceeded Motor Max');
            end
            
        end
    end
end

