function al = con_compute(x_bar,u_bar,al,pnts_gate,model)   
    %% Unpack and define some useful stuff
    N = size(x_bar,2);

    %% Compute constraint variables
    for k = 1:N
        % Compute Constraint value and its partials
        if k < N
            [al.con(1:8,k),al.con_x(1:8,:,k),al.con_u(1:8,:,k)] = input_con(u_bar(:,k),model);
        end
        [al.con(9:16,k),al.con_x(9:16,:,k),al.con_u(9:16,:,k)]    = gate_con(x_bar(:,k),pnts_gate,model.L_est,model.dt_fmu);
        [al.con(17:22,k),al.con_x(17:22,:,k),al.con_u(17:22,:,k)] = rate_con(x_bar(:,k));

        % Encode constraint activation
        for j = 1:16
            if al.con(j,k) <= 0 && al.lambda(j,k) == 0
                % Constraint not violated. Carry on.
            else
                % Constraint violated. Turn on Augment.
                al.I_mu(j,j,k) = al.mu(j,k);
            end
        end
        
    end
end