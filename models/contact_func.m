function [FT_ext,n_ct,model,targ] = contact_func(x_act,FT_ext,n_ct,model,targ,N_ct,contact_type)

% Compute location of grasper (world frame)
quat = x_act(7:10,1);  
bRw = quat2rotm(quat');

p_grasp_W = x_act(1:3) + bRw*model.p_grasp_b;

% check for contact
check_ct = targ.pos - p_grasp_W;

switch contact_type
    case 'grasp'
        if (norm(check_ct) < 0.3) && (n_ct == 0)      % First Contact! 
            % Build force array    
            m1 = model.m_act;
            m2 = targ.m_act;
            v1 = x_act(4:6,1);
            delta_v = m1*v1/(m1+m2);
            
            acc = delta_v/(N_ct*model.dt_act);
            F = -(m1+m2).*acc;
            FT_ext = [F ; cross(check_ct,F)];
    
            % Update Mass and Inertia
            d = model.p_grasp_b;
            model.I_act = model.I_act + diag(targ.m_act * d.^2);
            model.m_act = model.m_act + targ.m_act;
    
            n_ct = n_ct + 1;
            targ.t_capture = t_capture;

            disp('[main]: Contact triggered!');
        elseif (n_ct <= N_ct) && (n_ct > 0)
            n_ct = n_ct + 1;
        else
            FT_ext = zeros(6,1);
        end
    case 'perch'
        if (norm(check_ct) < 0.01)     % First Contact! 
            % Build force array    
            m1 = model.m_act;
            m2 = targ.m_act;
            v1 = x_act(4:6,1);
            
            acc = v1/(N_ct*model.dt_act);
            F = -(m1+m2).*acc;
            FT_ext = [F ; cross(check_ct,F)];
    
            n_ct = n_ct + 1;
            targ.t_capture = t_capture;

            disp('[main]: Contact triggered!');
        end
end


end