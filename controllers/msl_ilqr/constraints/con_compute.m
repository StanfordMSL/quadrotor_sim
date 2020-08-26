function [con,con_x,con_u] = con_compute(x_bar,u_bar,pnts_gate,model)   
    %% Unpack and define some useful stuff
    N = size(x_bar,2);
    n_x = size(x_bar,1);
    n_u = size(u_bar,1);
    
    con = zeros(16,N);
    con_x = zeros(16,n_x,N);
    con_u = zeros(16,n_u,N);

    %% Compute constraint variables
    for k = 1:N
        % Compute Constraint value and its partials
        if k < N
            [con(1:8,k),con_x(1:8,:,k),con_u(1:8,:,k)] = input_con(u_bar(:,k),model);
        end
        [con(9:16,k),con_x(9:16,:,k),con_u(9:16,:,k)]    = gate_con(x_bar(:,k),pnts_gate,model.L_est,model.dt_fmu);
        [con(17:22,k),con_x(17:22,:,k),con_u(17:22,:,k)] = rate_con(x_bar(:,k));        
    end
end