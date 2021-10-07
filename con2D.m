function g = con2D(x,u)
    % Obstacle Constraints
    p = [0;0];
    del_p = x(1:2)-p;
    g_pos = -del_p'*del_p+1;
    
    % Velocity Constraints
    g_vel = (x(4:5)'*x(4:5)) - 999;
    
    % Input Constraints
    g_u = u(1) - 999;
    
    % Total State/Input Constraints
    g = [g_pos ; g_vel ; g_u];
end