function [con,con_x,con_u] = con_compute(x,u_m,map,obj,model)   
    %% Compute constraint variables
    [con_r,con_r_x,con_r_u] = room_con(x,map);
    [con_g,con_g_x,con_g_u] = gate_con(x,obj,model);
    [con_m,con_m_x,con_m_u] = motor_limit_con(u_m,model);

    con   = [con_r ; con_g ; con_m];
    con_x = [con_r_x ; con_g_x ; con_m_x];
    con_u = [con_r_u ; con_g_u ; con_m_u];
end