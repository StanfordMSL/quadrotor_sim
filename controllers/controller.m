function [u,curr_m_cmd] = controller(x_now,k_con,nom,model,con_type)

switch con_type
    case 'ilqr'
        % Draw Out Motor Commands from u_bar computed by iLQR
        del_x = x_now-nom.x_bar(:,k_con);
        del_u = nom.alpha*nom.l(:,:,k_con) + nom.L(:,:,k_con)*del_x;
        u  = nom.u_bar(:,k_con) + del_u;
    case 'df'
        u = df_con(nom.f_out(:,:,k_con),model,'yaw');
end

curr_m_cmd = wrench2m_controller(u,model);
