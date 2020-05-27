function [u,curr_m_cmd] = fbc(x_now,x_bar,u_bar,l,L,alpha,model,motor_model)

% Draw Out Motor Commands from u_bar computed by iLQR
del_x = x_now-x_bar;
del_u = alpha*l + L*del_x;
u  = u_bar + del_u;

curr_m_cmd = wrench2m_controller(u,model);

switch motor_model
    case 'actual'
        for k = 1:4
            if curr_m_cmd(k,1) < model.motor_min
                curr_m_cmd(k,1) = model.motor_min;
            elseif curr_m_cmd(k,1) > model.motor_max
                curr_m_cmd(k,1) = model.motor_max;
            end
        end
    case 'ideal'
        % carry on
end