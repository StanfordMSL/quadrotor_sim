function [con,con_x,con_u] = motor_limit_con(u_bar,model)

% Unpack some stuff
kt =  model.kt_est(1,1);
w2m = model.m2w_inv;
omega_min = model.motor_min;
omega_max = model.motor_max;

% Count
N = size(u_bar,2) + 1;
n_u = size(u_bar,1);
n_x = 13;

% Min and Max Force
f_min = kt.*(omega_min.^2);
f_max = kt.*(omega_max.^2);

% Initialize Output
con   = zeros(2*n_u,N);
con_x = zeros(2*n_u,n_x,N);
con_u = zeros(2*n_u,n_u,N);

% % Compute Constraints and their partials
% for k = 1:N-1
%     % Individual Motor Values
%     f_m = w2m*u_bar(1:4,k);
%     
%     con_min  = -f_m + (f_min.*ones(4,1));
%     con_max  =  f_m - (f_max.*ones(4,1));
%     con(:,k) = [con_min ; con_max];
%     
%     con_u_min    = -w2m;
%     con_u_max    =  w2m;
%     con_u(:,:,k) = [con_u_min ; con_u_max];
% end

end