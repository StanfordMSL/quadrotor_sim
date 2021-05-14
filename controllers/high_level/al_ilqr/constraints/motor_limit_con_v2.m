function [con,con_x,con_u] = motor_limit_con_v2(x,u,model)

% Unpack some stuff
w_m_min = model.motor.min;
w_m_max = model.motor.max;
m  = model.est.m;
g  = model.est.g;
dt = model.est.dt;

% Count
N = size(u,2) + 1;
n_u = size(u,1);
n_x = 13;

% Initialize Output
con   = zeros(2*n_u,N);
con_x = zeros(2*n_u,n_x,N);
con_u = zeros(2*n_u,n_u,N);

% Compute Constraints and their partials
for k = 2:N-1    
    % Torque
    delta_w = x(2:4,k)-x(2:4,k-1);
    
    con_min  = -w_m + (w_m_min.*ones(4,1));
    con_max  =  w_m - (w_m_max.*ones(4,1));
    con(:,k) = [con_min ; con_max];
    
    con_u_min    = -eye(4);
    con_u_max    =  eye(4);
    con_u(:,:,k) = [con_u_min ; con_u_max];
end

end