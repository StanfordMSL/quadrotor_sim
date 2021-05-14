function [con,con_x,con_u] = con_compute(x,u,obj,input_mode,model)   
    %% Compute constraint variables
    [con_g,con_g_x,con_g_u] = gate_con(x,obj,model);
    
    switch input_mode
    case 'direct'
        [con_m,con_m_x,con_m_u] = motor_limit_con(u,model);
    case 'body_rate'
        [con_m,con_m_x,con_m_u] = motor_limit_con_v2(x,u,model);
    end

    con   = [con_g ; con_m];
    con_x = [con_g_x ; con_m_x];
    con_u = [con_g_u ; con_m_u];
end