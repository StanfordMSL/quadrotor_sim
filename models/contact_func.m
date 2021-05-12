function [FT_ext,obj,model] = contact_func(x_now,obj,model,contact_type)

% Compute location of grasper (world frame)
quat = x_now(7:10,1);  
bRw = quat2rotm(quat');

p_grasp_W = x_now(1:3) + bRw*model.grasp.pos;

% check for contact
check_ct = obj.x(1:3,2) - p_grasp_W;

switch contact_type
    case 'catch'
        if (norm(check_ct) < 0.3) && (obj.n_ct == 0)      % First Contact! 
            % Assign actual number of real-time frames
            obj.N_ct  = round(obj.dt_ct*model.clock.hz_act);  % Number of actual dynamics frames

            % Build force array    
            m1 = model.act.m;
            m2 = obj.m_act;
            v1 = x_now(4:6,1);
            delta_v = m1*v1/(m1+m2);
            
            acc = delta_v/(obj.N_ct*model.clock.dt_act);
            F = -(m1+m2).*acc;
            FT_ext = [F ; cross(check_ct,F)];    
            obj.FT_ext = FT_ext;
            
            % Update Mass and Inertia
            d = model.grasp.pos;
            model.act.I = model.act.I + diag(obj.m_act * d.^2);
            model.act.m = model.act.m + obj.m_act;
    
            obj.n_ct = obj.n_ct + 1;

            disp('[main]: Contact triggered!');
        elseif (obj.n_ct <= obj.N_ct) && (obj.n_ct > 0)
            FT_ext = obj.FT_ext;
            obj.n_ct = obj.n_ct + 1;
        else
            FT_ext = zeros(6,1);
        end
    case 'perch'
        if (norm(check_ct) < 0.01)     % First Contact! 
            % Build force array    
            m1 = model.act.m;
            m2 = obj.m_act;
            v1 = x_now(4:6,1);
            
            acc = v1/(N_ct*model.clock.dt_act);
            F = -(m1+m2).*acc;
            FT_ext = [F ; cross(check_ct,F)];
    
            obj.n_ct = obj.n_ct + 1;

            disp('[main]: Contact triggered!');
        end
end


end