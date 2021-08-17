function [c, cx, cu] = conu_calc(u,up)

% Initialize Variables
n_u   = size(u,1);
N     = size(u,2);

c  = zeros(8,N+1);
cx = zeros(8,10,N+1);
cu = zeros(8,n_u,N+1);

for k = 1:N
    u_k = u(:,k);
    up_k = up(:,k);
    
    c(1:8,k)     = conu(u_k,up_k);
    cu(1:8,:,k)  = conu_u(u_k,up_k);
end

end