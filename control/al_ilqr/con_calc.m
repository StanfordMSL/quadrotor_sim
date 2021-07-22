function con = con_calc(x,u,p_box)

% Initialize Variables
n_x   = 10;
n_u   = size(u,1);
N     = size(x,2);

c   = zeros(24,N);
cx = zeros(24,n_x,N);
cu = zeros(24,n_u,N-1);

p12 = p_box(:,2)-p_box(:,1);
p14 = p_box(:,4)-p_box(:,1);
plane = [ cross(p12,p14) ; dot(cross(p12,p14),p_box(:,1))];

for k = 1:N-1
    x_k = x(:,k);
    u_k = u(:,k);
    if k == 1
        u_p = zeros(4,1);
    else
        u_p = u(:,k-1);
    end
    
%     c(1:8,k)     = motor_con(x_k,u_k,u_p);
%     cx(1:8,:,k)  = motor_con_x(x_k,u_k,u_p);
%     cu(1:8,:,k)  = motor_con_u(x_k,u_k,u_p);

    dist = plane_dist(plane,x_k(1:3));
    if dist < 0.1
        c(9:24,k)    = gate_con(x_k,u_k,p_box);   
        cx(9:24,:,k) = gate_con_x(x_k,u_k,p_box);
        cu(9:24,:,k) = gate_con_u(x_k,p_box);
    end
end

x_k = x(:,N);
    dist = plane_dist(plane,x_k(1:3));
if dist < 0.1
    c(9:24,N)    = gate_con(x_k,u_k,p_box);
    cx(9:24,:,N) = gate_con_x(x_k,u_k,p_box);
end

con.c  = c;
con.cx = cx;
con.cu = cu;

end