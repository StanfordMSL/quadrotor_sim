function m_cmd_curr = direct_motor_control(u,model)

% Compute Motor Commands
m_cmd_curr = wrench2m_cmd(u,model);

for k = 1:4
    if m_cmd_curr(k,1) < model.motor_min
        m_cmd_curr(k,1) = model.motor_min;

        if m_cmd_curr(k,1) <  (model.motor_min - 1e-6)
            disp('[ilqr]: Exceeded Motor Min');
        end

    elseif m_cmd_curr(k,1) > model.motor_max
        m_cmd_curr(k,1) = model.motor_max;

        if m_cmd_curr(k,1) > (model.motor_max + 1e-6)
            disp('[ilqr]: Exceeded Motor Max');
        end

    end
end