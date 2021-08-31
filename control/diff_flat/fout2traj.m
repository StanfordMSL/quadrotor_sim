function traj = fout2traj(traj,n_tr,f_out,model,mode)

% Unpack some stuff
dt_fmu = 1/traj.hz;
N_tr = size(f_out,3); 

% Some Useful Terms
fn = f2fn(model.motor.thrust_hover);

%%% Full State Nominal Trajectory
x_bar      = zeros(13,N_tr);
x_bar(:,1) = traj.x_bar(:,n_tr);

%%% Body Rates
x_br      = zeros(17,N_tr);
u_br      = zeros(4,N_tr-1);
L_br      = zeros(4,10,N_tr-1);

x_br(1:10,1) = traj.x_bar(1:10,n_tr);
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
    
    % Output to Motors
    u_mt(:,k) = w2m(u_wr(:,k));
    
    % Full State Nominal Trajectory
    FT_ext = zeros(6,1);
    wt = zeros(13,1);
    x_bar(:,k+1) = quadcopter_est(x_bar(:,k),u_mt(:,k),FT_ext,wt);
    
    % Body Rates
    x_br(1:10,k) = x_bar(1:10,k);
    u_br(:,k)    = [f2fn(u_wr(1,k)) ; x_bar(11:13,k)];
    L_br(:,:,k)  = zeros(4,10);
end
x_br(1:10,N_tr)   = x_bar(1:10,N_tr);

% Store it in the desired format
traj.t_fmu = 0:dt_fmu:(N_tr-1)*dt_fmu;
traj.x_bar = x_bar;

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