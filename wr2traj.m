function traj = wr2traj(U,obj,model,hz)

% Unpack some stuff
dt_fmu = 1/hz;
N = size(U,2)+1; 

%%% Full State Nominal Trajectory
X      = zeros(13,N);
X(:,1) = obj.x(:,1);

% Run the trajectory forward to generate the various terms
for k = 1:N-1
    % Nominal Parameter Input
    theta = [model.m ; model.Ipp ; model.Df(1:3,1)];
    
    % Full State Nominal Trajectory
    X(:,k+1) = quad_sim(X(:,k),U(:,k),theta,dt_fmu);
end

% Store it in the desired format
traj.t_fmu = 0:dt_fmu:(N-1)*dt_fmu;
traj.X = X;
traj.U = U;
