function [c, cx, cu] = conu_calc(u,up,n_x,n_u,N)

% Initialize Variables
c  = zeros(8,N);
cx = zeros(8,n_x,N);
cu = zeros(8,n_u,N);

for k = 1:N-1
    u_k = u(:,k);
    up_k = up(:,k);
    
    c(1:8,k)     = conu(u_k,up_k);
    cu(1:8,:,k)  = conu_u(u_k,up_k);
end

end