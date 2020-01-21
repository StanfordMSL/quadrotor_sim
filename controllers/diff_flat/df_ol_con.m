function [u,curr_m_cmd] = df_ol_con(k_ctl,nom,model,motor_model)

u = df_con(nom.f_out(:,:,k_ctl),model,'yaw');

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