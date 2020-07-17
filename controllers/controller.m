function [u,curr_m_cmd] = controller(x_now,x_bar,l,L,model,con_type,motor_model)

switch con_type
    case 'ilqr'
        % Draw Out Motor Commands from u_bar computed by iLQR
        del_x = x_now-x_bar;
        del_u = l + L*del_x;
        u  = nom.u_bar(:,k_con) + del_u;
    case 'df'
        u = df_con(nom.f_out(:,:,k_con),model,'yaw');
end

curr_m_cmd = wrench2m_controller(u,model);

switch motor_model
    case 'actual'
        for k = 1:4
            if curr_m_cmd(k,1) < model.motor_min
                curr_m_cmd(k,1) = 0;
            elseif curr_m_cmd(k,1) > model.motor_max
                curr_m_cmd(k,1) = model.motor_max;
            end
        end
    case 'ideal'
        % carry on
end