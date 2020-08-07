function [J,J_stage] = J_calc(x_star,x_bar,u_bar,wts)
    % Unpack some stuff
    N = size(x_bar,2);
    
    Q_t = wts.Q_pstn;
    Q_f = wts.Q_unif;
    R   = wts.R_stnd;
    
    % Initialize cost variables
    J_stage = zeros(1,N);
    % Compute stagewise cost
    for k = 1:N-1
        err_x = x_bar(:,k) - x_star;
        err_u = u_bar(:,k);
        J_stage(1,k) = 0.5* err_x'*Q_t*err_x + 0.5*err_u'*R*err_u;
    end
    
    % Terminal Case
    err_x = x_bar(:,end) - x_star;
    J_stage(1,N) = 0.5* err_x'*Q_f*err_x;
    
    % Total
    J = sum(J_stage);
end