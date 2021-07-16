function con = con_calc(x,u)

% Initialize Variables
n_x   = 10;
n_u   = size(u,1);
N     = size(x,2);

c   = zeros(24,N);
cx = zeros(24,n_x,N);
cu = zeros(24,n_u,N-1);

for k = 1:N-1
    x_k = x(:,k);
    u_k = u(:,k);
    if k == 1
        u_p = zeros(4,1);
    else
        u_p = u(:,k-1);
    end
    
    c(1:8,k)  = motor_con(x_k,u_k,u_p);
    cx(1:8,:,k)  = motor_con_x(x_k,u_k,u_p);
    cu(1:8,:,k)  = motor_con_u(x_k,u_k,u_p);

    if abs(x_k(1,1)) < 0.2
        c(9:24,k) = gate_con(x_k,u_k);   
        cx(9:24,:,k) = gate_con_x(x_k,u_k);
        cu(9:24,:,k) = gate_con_u(x_k,u_k);
    end
end

x_k = x(:,N);
if abs(x_k(1,1)) < 0.2
    c(9:24,N)    = gate_con(x_k,u_k);
    cx(9:24,:,N) = gate_con_x(x_k,u_k);
end

con.c  = c;
con.cx = cx;
con.cu = cu;

end