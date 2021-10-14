function x_dot = quad2D(x,u)
    th  =  x(3);
    v   = [x(4) ; x(5)];   % Linear Velocity
    thd =  x(6);            % Angular Velocity

    % Generate Equations %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % Unpack
    %         dt    = db.dt;
    g     = 9.81;
    m     = 0.53;
    Iqd   = 0.0016 ;

    % Forces
    F_g   = m.*[0 ; -g];                        % Gravity
    F_t   = [-u(1)*sin(th) ; u(1)*cos(th)];     % Thrust

    % Torques
    tau_wr   = u(2);                            % Torque Input

    % Dynamics Equations
    p_dot   = v;
    th_dot  = thd;
    v_dot   = (1/m) .* ( F_g + F_t);
    thd_dot = tau_wr/Iqd;

    x_dot = [ p_dot ; th_dot ; v_dot ; thd_dot];
end