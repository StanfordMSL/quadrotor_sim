function [FT_ext_arr, model, targ] = contact_func(model,targ,t_capture,x_act,check_ct,N_ct)
    
    % Build force array    
    m1 = model.m_act;
    m2 = targ.m_act;
    v1 = x_act(4:6,1);
    a = (-m1/(N_ct*(m1+m2))).*v1;
    
    F = (m1+m2).*a;
    FT_ext_arr = repmat([F ; cross(check_ct,F)],1,N_ct);
    
    % Update Mass and Inertia
    d = [model.leg_l ; model.leg_l ; 0];
    model.I_act = model.I_act + diag(targ.m_act * d.^2);
    model.m_act = model.m_act + targ.m_act;
    targ.t_capture = t_capture;
end