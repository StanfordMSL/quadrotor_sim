function traj = fout2traj(traj,n_tr,f_out,model,mode)

% Number of frames in this portion of the trajectory
N_tr = size(f_out,3); 

% Some Useful Terms
fn = f2fn(model.motor.thrust_hover);

%%% Full State Nominal Trajectory
x_bar      = zeros(13,N_tr);
x_bar(:,1) = traj.x_bar(:,n_tr);

%%% Body Rates
x_br      = zeros(10,N_tr);
u_br      = zeros(4,N_tr-1);
L_br      = zeros(4,10,N_tr-1);

x_br(:,1) = traj.x_bar(1:10,n_tr);
u_br(:,1) = [fn ; traj.x_bar(11:13,n_tr)];

%%% Direct
u_mt = zeros(4,N_tr-1);

%%% Wrench
u_wr = zeros(4,N_tr-1);

% Initialize the PosAtt Controller
pa = pa_init();

% Run the trajectory forward to generate the various terms
for k = 1:N_tr-1
    % Wrench through Pos Att
    u_wr(:,k) = pa_ctrl(x_bar(:,k),f_out(:,:,k),pa,model.est);
%     u_wr(:,k) = df_con(f_out(:,:,k),model.est);
    
    % Output to Motors
    u_mt(:,k) = w2m_est(u_wr(:,k));
    
    % Full State Nominal Trajectory
    FT_ext = zeros(6,1);
    wt = zeros(13,1);
    x_bar(:,k+1) = quadcopter_est(x_bar(:,k),u_mt(:,k),FT_ext,wt);
    
    % Body Rates
    x_br(:,k)   = x_bar(1:10,k);
    u_br(:,k)   = [f2fn(u_wr(1,k)) ; x_bar(11:13,k)];
    L_br(:,:,k) = zeros(4,10);
end
x_br(:,N_tr)   = x_bar(1:10,N_tr);

% Store it in the desired format
idx_x = n_tr:n_tr+N_tr-1;
idx_u = n_tr:n_tr+N_tr-2;

traj.x_bar(:,idx_x) = x_bar;
switch mode
    case 'pos_att'
        traj.f_out(:,:,idx_x) = f_out;
    case 'body_rate'
        traj.x_br(:,idx_x) = x_br;
        traj.u_br(:,idx_u) = u_br;
        traj.L_br(:,:,idx_u) = L_br;
    case 'direct'
        traj.u_mt(:,idx_u) = u_mt;
    case 'wrench'
        traj.u_wr(:,idx_u) = u_wr;
end

traj.T = n_tr + N_tr - 1;

end