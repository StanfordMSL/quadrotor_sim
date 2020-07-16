function del_u = del_u_update(l,L)

    % 
%     x_fmu = x_curr;
%     J =  0;
%     k_fmu = 1;
%     k_ctl = 1;
%     
%     t_cmd = u_cmd(5,1);
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%         % Feedback Law Updater
%         if t_now >= t_cmd
%             % Update Pointer
%             k_ctl = k_ctl + 1;
%             
%             % Generate new motor command
%             [u_cmd,m_cmd] = fbc(x_fmu,x_bar(:,k_ctl),u_bar(:,k_ctl),l(:,:,k_ctl),L(:,:,k_ctl),model,'ideal');
% 
%             % Determine next time to update pointer
%             t_cmd = t_cmd + u_cmd(5,1);
%             
%             % Log Current States
%             x_bar(1:13,k_ctl) = x_fmu;
%             u_bar(:,k_ctl)    = u_cmd;
% 
%             % Calculate new costs
%             del_x = x_bar(:,k_ctl) - x_wp;
%             J = J + 0.5*(del_x'*Q_t*del_x + u_cmd'*R*u_cmd);
%         end  

    % Actual forward pass
    while k_ctl < N
        % Check if new fmu time has passed controller time marker
        if t_fmu >= t_ctl
            % If so, extract motor command
            if init_flag == 1
                % Initial case
                del_u = zeros(5,1);
            else
                % Normal case
                del_x = x_fmu - x_bar(:,k_ctl);
                del_u = model.alpha.*(l(:,:,k_ctl) + L(:,:,k_ctl)*del_x);
            end        
            u_curr = u_curr + del_u;
            m_cmd = wrench2m_controller(u_bar(1:4,k_ctl),model);
            
            % Update controller time marker             
            dt_step = u_curr(5,1)^2;
            if dt_step < model.dt_fmu
               dt_step = model.dt_fmu;
            end
            t_ctl = t_ctl + dt_step;
            
            % Update pointer
            k_ctl = k_ctl + 1;
        end
       
        % Run model
        x_fmu = quadcopter(x_fmu,m_cmd,model,FT_ext,'fmu');
        
        % Update fmu
        t_fmu = (k_fmu * dt_fmu);
    end