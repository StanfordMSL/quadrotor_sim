function traj = fout2traj(traj,n_tr,f_out,model,mode)

% Unpack some stuff
dt_fmu = 1/traj.hz;
N_tr = size(f_out,3); 

%%% Full State Nominal Trajectory
x_act      = zeros(13,N_tr);
x_act(1:10,1) = traj.x_br(1:10,n_tr);

%%% Body Rates
x_br      = zeros(17,N_tr);
u_br      = zeros(4,N_tr-1);
L_br      = zeros(4,17,N_tr-1);

%%% Direct
u_mt = zeros(4,N_tr-1);

%%% Wrench
u_wr = zeros(4,N_tr-1);

% Initialize the PosAtt Controller
pa = pa_init();

% Run the trajectory forward to generate the various terms
for k = 1:N_tr-1
    % Wrench through Pos Att
    u_wr(:,k) = pa_ctrl(x_act(:,k),f_out(:,:,k),pa,model.est);
%     u_wr(:,k) = df_con(f_out(:,:,k),model.est);
    
    % Full State Nominal Trajectory
    x_act(:,k+1) = quadcopter_df(x_act(:,k),u_wr(:,k));
    
    % Body Rates
    x_br(1:10,k) = x_act(1:10,k);
    u_br(:,k)    = [f2fn(u_wr(1,k)) ; x_act(11:13,k)];
    L_br(:,:,k)  = zeros(4,17);
end
x_br(1:10,N_tr)   = x_act(1:10,N_tr);

% Store it in the desired format
traj.t_fmu = 0:dt_fmu:(N_tr-1)*dt_fmu;

switch mode
    case 'pos_att'
        traj.f_out = f_out;
    case 'body_rate'
        traj.x_br = x_br;
        traj.u_br = u_br;
        traj.L_br = L_br;
    case 'direct'
        traj.u_mt = u_mt;
    case 'wrench'
        traj.u_wr = u_wr;
end

end