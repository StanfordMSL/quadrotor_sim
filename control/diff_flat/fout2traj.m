function traj = fout2traj(f_out,model,x0,hz)

% Unpack some stuff
dt_fmu = 1/hz;
N_tr = size(f_out,3); 

%%% Full State Nominal Trajectory
X      = zeros(13,N_tr);
X(:,1) = x0;

%%% Wrench
U = zeros(4,N_tr-1);

% Run the trajectory forward to generate the various terms
for k = 1:N_tr-1
    % Wrench through Pos Att
    U(:,k) = pa_ctrl(X(:,k),f_out(:,:,k),model);

    % Nominal Parameter Input
    theta = [model.m ; model.Ipp ; model.Df(1:3,1)];
    
    % Full State Nominal Trajectory
    X(:,k+1) = quad_sim(X(:,k),U(:,k),theta,dt_fmu);
end

% Store it in the desired format
traj.t_fmu = 0:dt_fmu:(N_tr-1)*dt_fmu;
traj.X = X;
traj.U = U;
