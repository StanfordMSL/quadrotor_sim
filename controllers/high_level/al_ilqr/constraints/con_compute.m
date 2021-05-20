function [con,con_x,con_u] = con_compute(x,u,input_mode)

% Sizes
n_x = size(x,1);
n_u = size(u,1);
N   = size(x,2);

n_g = size((gate_con(x(:,1),u(:,1))),1);
n_m = 8;
n_con = n_m + n_g;

con   = zeros(n_con,N);
con_x = zeros(n_con,n_x,N);
con_u = zeros(n_con,n_u,N);

%% Compute constraint variables
for k = 1:N
    x_c = x(:,k);
    if k == N
        u_c = u(:,k-1);
    else
        u_c = u(:,k);
    end
    
    if k == 1
        u_p = u_c;
    else
        u_p = u(:,k-1);
    end
    
    switch input_mode
    case 'direct'
        con_m   = motor_con(x_c,u_c);
        con_m_x = motor_con_x(x_c,u_c);
        con_m_u = motor_con_u(x_c,u_c);
    case 'body_rate'
        con_m   = motor_con(x(:,k),u_c,u_p);
        con_m_x = motor_con_x(x(:,k),u_c,u_p);
        con_m_u = motor_con_u(x(:,k),u_c,u_p);
    end
    
    if ( abs(x(1,k)) < 0.05 ) 
%         disp(k)
        con_g   = gate_con(x_c,u_c);
        con_g_x = gate_con_x(x_c,u_c);
        con_g_u = gate_con_u(x_c,u_c);
    else
        con_g   = zeros(n_g,1);
        con_g_x = zeros(n_g,n_x);
        con_g_u = zeros(n_g,n_u);
    end
    
    con(:,k)   = [con_g ; con_m];
    con_x(:,:,k) = [con_g_x ; con_m_x];
    con_u(:,:,k) = [con_g_u ; con_m_u];
end

end