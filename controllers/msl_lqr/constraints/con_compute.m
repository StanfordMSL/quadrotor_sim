function [con,con_x,con_u] = con_compute(x_bar,u_bar,obj,model)   
    %% Compute constraint variables
    [con_r,con_r_x,con_r_u] = room_con(x_bar,obj);
    [con_g,con_g_x,con_g_u] = gate_con(x_bar,obj,model);
    [con_m,con_m_x,con_m_u] = motor_limit_con(u_bar,model);

    con   = [con_r ; con_g ; con_m];
    con_x = [con_r_x ; con_g_x ; con_m_x];
    con_u = [con_r_u ; con_g_u ; con_m_u];
end