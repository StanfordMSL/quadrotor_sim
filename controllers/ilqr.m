function [m_cmd,nom] = ilqr(t_now,x_fc,nom,wp,fc,model)
    nom.index = nom.index + 1;
    % Determine current point along trajectory and remainder of points
    n = find(nom.t_bar > t_now,1)-1;
    N = nom.total-n+1;
    
    % Unpack the Terms    
    x_bar = nom.x_bar(:,n:end);
    u_bar = nom.u_bar(:,n:end);  

    % Initialize the iLQR variables
    l = zeros(4,1,N);
    L = zeros(4,12,N);

    % Convergence variables
    itrs = 0;
    del_u_tol = 10;
    
    while del_u_tol > 1e-1
        itrs = itrs + 1;

        % Forward Pass
        [x_bar,u_bar,del_u,A,B] = ilqr_fp(x_bar,u_bar,t_now,x_fc,wp,l,L,N,model,fc);

        % Backward Pass   
        [l,L] = ilqr_bp(x_bar,u_bar,t_now,wp,A,B,N,model,fc);

        % Check for Convergence
        if itrs == 1
            % Do nothing
        elseif itrs > 1 && itrs < 1000
            del_u_tol = sum(vecnorm(del_u))/N;
        else
            x_bar = nom.x_bar(:,n:end);
            u_bar = nom.u_bar(:,n:end);
            disp('[ilqr]: convergence timeout');
            break;
        end
    end
    
    nom.x_bar(:,n:end) = x_bar;
    nom.u_bar(:,n:end) = u_bar;

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

