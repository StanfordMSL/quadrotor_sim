function [traj,n_tr] = fout2traj(traj,n_tr,f_out,model,mode)

% Number of frames in this portion of the trajectory
N_tr = size(f_out,3); 

% Some Useful Terms
f_hov = model.motor.thrust_hover;
f_max = model.motor.thrust_max;
fn = f_hov/f_max;

%%% Full State Nominal Trajectory
x_bar      = zeros(13,N_tr);
x_bar(:,1) = traj.x_bar(:,n_tr);

%%% Body Rates
x_br      = zeros(10,N_tr);
l_br      = zeros(4,N_tr-1);
L_br      = zeros(4,10,N_tr-1);

x_br(:,1) = traj.x_bar(1:10,n_tr);
l_br(:,1) = [fn ; traj.x_bar(11:13,n_tr)];

%%% Direct
u_mt = zeros(4,N_tr-1);

%%% Wrench
u_wr = zeros(4,N_tr-1);

% Run the trajectory forward to generate the various terms
for k = 1:N_tr-1
    % Wrench
    u_wr(:,k) = df_con(f_out(:,:,k),model.est);
    
    % Direct
    T_motor = model.est.w2m*u_wr(:,k);
    u_mt(:,k) = sqrt(T_motor./model.est.kw(1,1));
    
    % Full State Nominal Trajectory
    FT_ext = zeros(6,1);
    wt = zeros(13,1);
    x_bar(:,k+1) = quadcopter_est(x_bar(:,k),u_mt(:,k),FT_ext,wt);
    
    % Body Rates
    x_br(:,k)   = x_bar(1:10,k);
    l_br(:,k)   = [f2fn(u_wr(1,k)) ; x_bar(11:13,k)];
    L_br(:,:,k) = zeros(4,10);
end

% Store it in the desired format
idx_x = n_tr:n_tr+N_tr-1;
idx_u = n_tr:n_tr+N_tr-2;

traj.x_bar(:,idx_x) = x_bar;
switch mode
    case 'pos_att'
        traj.f_out(:,:,idx_x) = f_out;
    case 'body_rate'
        traj.x_br(:,idx_x) = x_br;
        traj.l_br(:,idx_u) = l_br;
    case 'direct'
        traj.u_mt(:,idx_u) = u_mt;
    case 'wrench'
        traj.u_wr(:,idx_u) = u_wr;
end

n_tr = n_tr + N_tr - 1;

end