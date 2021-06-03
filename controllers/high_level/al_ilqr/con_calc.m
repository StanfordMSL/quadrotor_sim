function [con,con_x,con_u] = con_calc(x,u)

% Initialize Variables
n_x   = 10;
n_u   = size(u,1);
N     = size(x,2);

con   = zeros(24,N);
con_x = zeros(24,n_x,N);
con_u = zeros(24,n_u,N-1);

for k = 1:N-1
    x_k = x(:,k);
    u_k = u(:,k);
    if k == 1
        u_p = zeros(4,1);
    else
        u_p = u(:,k-1);
    end
    
%     con(1:8,k)  = motor_con(x_k,u_k,u_p);
%     con_x(1:8,:,k)  = motor_con_x(x_k,u_k,u_p);
%     con_u(1:8,:,k)  = motor_con_u(x_k,u_k,u_p);

    if abs(x_k(1,1)) < 0.2
        con(9:24,k) = gate_con(x_k,u_k);   
        con_x(9:24,:,k) = gate_con_x(x_k,u_k);
        con_u(9:24,:,k) = gate_con_u(x_k,u_k);
    end
end

x_k = x(:,N);
if abs(x_k(1,1)) < 0.2
    con(9:24,N)     = gate_con(x_k,u_k);
    con_x(9:24,:,N) = gate_con_x(x_k,u_k);
end
end