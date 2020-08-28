function al = al_init(traj,obj,model)

% Count
N_tot = size(traj.x,2);
n_con = 30;

% Initialize Lagrange Multiplier Variables
al.mu     = ones(n_con,N_tot);
al.lambda = 0.*ones(n_con,N_tot);
al.phi    = 2.0.*ones(n_con,1);

% Compute Constraints and their Partials
[al.con,al.con_x,al.con_u] = con_compute(traj.x,traj.u,obj,model);

% Update the trigger matrices
al.I_mu = con_trigger(al.con,al.lambda,al.mu);


end