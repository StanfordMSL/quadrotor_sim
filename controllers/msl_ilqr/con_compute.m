function con_struct = con_compute(x_bar,u_bar,al,pnts_gate,model)   
    %% Unpack and define some useful stuff
    N = size(x_bar,2);
    lambda = al.lambda;
    mu = al.mu;
    
    %% Initialize constraints
    con   = -999.*ones(8,N);
    con_x = zeros(8,13,N);
    con_u = zeros(8,4,N);
    I_mu  = zeros(16,16,N);

    %% Compute constraint variables
    for k = 1:N
        % Compute Constraint value and its partials
        if k < N
            [con(1: 8,k),con_x(1: 8,:,k),con_u(1: 8,:,k)] = motor_con(u_bar(:,k),model.kt_act(1,1),model.m2w_inv,model.motor_min,model.motor_max);
        end
        [con(9:16,k),con_x(9:16,:,k),con_u(9:16,:,k)] = gate_con(x_bar(:,k),pnts_gate,model.L_est,model.dt_fmu);
        
        % Encode constraint activation
        for j = 1:16
            if con(j,1) <= 0 && lambda(j,k) == 0
                % Constraint not violated. Carry on.
            else
                % Constraint violated. Turn on Augment.
                I_mu(j,j,k) = mu(j,k);
            end
        end
    end

    % Package them
    con_struct.con   = con;
    con_struct.con_x = con_x;
    con_struct.con_u = con_u;
    con_struct.I_mu  = I_mu;
end