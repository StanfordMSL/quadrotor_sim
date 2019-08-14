function inputs = tvlqr(mu_curr,u_curr,x_nom,craft)
    % Unpack
    model = craft.model;
    hover_u = craft.con.hover_u;
    Q = craft.con.Q;
    R = craft.con.R;
    
    % Compute A and B
    A = A_calc(mu_curr,u_curr,model);   
    B = B_calc(mu_curr,u_curr,model);
    
    % Compute K
    [K,~,~] = dlqr(A,B,Q,R);
    
    % Compute Motor Commands
    inputs = -K*(mu_curr-x_nom)+hover_u;
    
    for k = 1:4
        if inputs(k,1) < model.motor_min
            inputs(k,1) = model.motor_min;
        end
        if inputs(k,1) > model.motor_max
            inputs(k,1) = model.motor_max;
        end
    end
end