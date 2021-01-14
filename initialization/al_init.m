function al = al_init(traj,obj,model)

% Count
N_tot = size(traj.x,2);
n_con = 30;

% Initialize Lagrange Multiplier Variables
al.mu     = [ 1.00.*ones( 6,N_tot) ;...     % room
              1.00.*ones(16,N_tot) ;...     % gate
              1.00.*ones( 8,N_tot) ];       % motor
al.lambda = 0.*ones(n_con,N_tot);
al.phi    = 2.*ones(n_con,1);

% Compute Constraints and their Partials
[al.con,al.con_x,al.con_u] = con_compute(traj.x,traj.u,obj,model);

% Update the trigger matrices
al.I_mu = con_trigger(al.con,al.lambda,al.mu);


end